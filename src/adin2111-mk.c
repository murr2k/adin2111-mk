// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/* ADIN1110 Low Power 10BASE-T1L Ethernet MAC-PHY
 * ADIN2111 2-Port Ethernet Switch with Integrated 10BASE-T1L PHY
 *
 * Copyright 2021 Analog Devices Inc.
 * 
 * ADIN2111-MK Single Ethernet Interface Driver:
 * Simplified single-port driver derived from ADIN2111 2-Port Ethernet Switch
 * 
 * Author: Murray Kopit <murr2k@gmail.com>
 * 
 * Modifications for single interface operation:
 * - Removed dual-port switching logic
 * - Removed bridge/switchdev dependencies 
 * - Simplified to single eth0 interface
 * - Optimized for direct ethernet operations
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cache.h>
#include <linux/crc8.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/regulator/consumer.h>
#include <linux/phy.h>
#include <linux/property.h>
#include <linux/spi/spi.h>

#include <asm/unaligned.h>

#define ADIN1110_PHY_ID				0x1

#define ADIN1110_RESET				0x03
#define   ADIN1110_SWRESET			BIT(0)

#define ADIN1110_CONFIG1			0x04
#define   ADIN1110_CONFIG1_SYNC			BIT(15)

#define ADIN1110_CONFIG2			0x06
#define   ADIN1110_CRC_APPEND			BIT(5)

#define ADIN1110_STATUS0			0x08

#define ADIN1110_STATUS1			0x09
#define   ADIN1110_SPI_ERR			BIT(10)
#define   ADIN1110_RX_RDY			BIT(4)

#define ADIN1110_IMASK1				0x0D
#define   ADIN1110_SPI_ERR_IRQ			BIT(10)
#define   ADIN1110_RX_RDY_IRQ			BIT(4)
#define   ADIN1110_TX_RDY_IRQ			BIT(3)

#define ADIN1110_MDIOACC			0x20
#define   ADIN1110_MDIO_TRDONE			BIT(31)
#define   ADIN1110_MDIO_ST			GENMASK(29, 28)
#define   ADIN1110_MDIO_OP			GENMASK(27, 26)
#define   ADIN1110_MDIO_PRTAD			GENMASK(25, 21)
#define   ADIN1110_MDIO_DEVAD			GENMASK(20, 16)
#define   ADIN1110_MDIO_DATA			GENMASK(15, 0)

#define ADIN1110_TX_FSIZE			0x30
#define ADIN1110_TX				0x31
#define ADIN1110_TX_SPACE			0x32

#define ADIN1110_MAC_ADDR_FILTER_UPR		0x50
#define   ADIN1110_MAC_ADDR_APPLY2PORT		BIT(30)
#define   ADIN1110_MAC_ADDR_TO_HOST		BIT(16)

#define ADIN1110_MAC_ADDR_FILTER_LWR		0x51

#define ADIN1110_MAC_ADDR_MASK_UPR		0x70
#define ADIN1110_MAC_ADDR_MASK_LWR		0x71

#define ADIN1110_RX_FSIZE			0x90
#define ADIN1110_RX				0x91

#define ADIN1110_CLEAR_STATUS0			0xFFF

/* MDIO_OP codes */
#define ADIN1110_MDIO_OP_WR			0x1
#define ADIN1110_MDIO_OP_RD			0x3

#define ADIN1110_CD				BIT(7)
#define ADIN1110_WRITE				BIT(5)

#define ADIN1110_MAX_BUFF			2048
#define ADIN1110_MAX_FRAMES_READ		64
#define ADIN1110_WR_HEADER_LEN			2
#define ADIN1110_FRAME_HEADER_LEN		2
#define ADIN1110_INTERNAL_SIZE_HEADER_LEN	2
#define ADIN1110_RD_HEADER_LEN			3
#define ADIN1110_REG_LEN			4
#define ADIN1110_FEC_LEN			4

#define ADIN2111_PHY_ID_VAL			0x0283BCA1

DECLARE_CRC8_TABLE(adin1110_crc_table);

enum adin1110_chips_id {
	ADIN2111_MAC_MK = 0,
};

struct adin1110_cfg {
	enum adin1110_chips_id	id;
	char			name[MDIO_NAME_SIZE];
	u32			phy_id;
	u32			phy_id_val;
};

struct adin2111_mk_priv {
	struct mutex			lock; /* protect spi */
	struct mii_bus			*mii_bus;
	struct spi_device		*spidev;
	struct net_device		*netdev;
	struct phy_device		*phydev;
	bool				append_crc;
	struct adin1110_cfg		*cfg;
	u32				tx_space;
	u32				irq_mask;
	int				irq;
	struct work_struct		tx_work;
	struct work_struct		rx_mode_work;
	u64				rx_packets;
	u64				tx_packets;
	u64				rx_bytes;
	u64				tx_bytes;
	u32				flags;
	struct sk_buff_head		txq;
	char				mii_bus_name[MII_BUS_ID_SIZE];
	u8				data[ADIN1110_MAX_BUFF] ____cacheline_aligned;
};

static struct adin1110_cfg adin2111_mk_cfg = {
	.id = ADIN2111_MAC_MK,
	.name = "adin2111-mk",
	.phy_id = 1,
	.phy_id_val = ADIN2111_PHY_ID_VAL,
};

static u8 adin1110_crc_data(u8 *data, u32 len)
{
	return crc8(adin1110_crc_table, data, len, 0);
}

static int adin1110_read_reg(struct adin2111_mk_priv *priv, u16 reg, u32 *val)
{
	u32 header_len = ADIN1110_RD_HEADER_LEN;
	u32 read_len = ADIN1110_REG_LEN;
	struct spi_transfer t = {0};
	int ret;

	priv->data[0] = ADIN1110_CD | FIELD_GET(GENMASK(12, 8), reg);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), reg);
	priv->data[2] = 0x00;

	if (priv->append_crc) {
		priv->data[2] = adin1110_crc_data(&priv->data[0], 2);
		priv->data[3] = 0x00;
		header_len++;
	}

	if (priv->append_crc)
		read_len++;

	memset(&priv->data[header_len], 0, read_len);
	t.tx_buf = &priv->data[0];
	t.rx_buf = &priv->data[0];
	t.len = read_len + header_len;

	ret = spi_sync_transfer(priv->spidev, &t, 1);
	if (ret)
		return ret;

	if (priv->append_crc) {
		u8 recv_crc;
		u8 crc;

		crc = adin1110_crc_data(&priv->data[header_len],
					ADIN1110_REG_LEN);
		recv_crc = priv->data[header_len + ADIN1110_REG_LEN];

		if (crc != recv_crc) {
			dev_err_ratelimited(&priv->spidev->dev, "CRC error.");
			return -EBADMSG;
		}
	}

	*val = get_unaligned_be32(&priv->data[header_len]);

	return ret;
}

static int adin1110_write_reg(struct adin2111_mk_priv *priv, u16 reg, u32 val)
{
	u32 header_len = ADIN1110_WR_HEADER_LEN;
	u32 write_len = ADIN1110_REG_LEN;

	priv->data[0] = ADIN1110_CD | ADIN1110_WRITE | FIELD_GET(GENMASK(12, 8), reg);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), reg);

	if (priv->append_crc) {
		priv->data[2] = adin1110_crc_data(&priv->data[0], header_len);
		header_len++;
	}

	put_unaligned_be32(val, &priv->data[header_len]);
	if (priv->append_crc) {
		priv->data[header_len + write_len] = adin1110_crc_data(&priv->data[header_len],
								       write_len);
		write_len++;
	}

	return spi_write(priv->spidev, &priv->data[0], header_len + write_len);
}

static int adin1110_set_bits(struct adin2111_mk_priv *priv, u16 reg,
			     unsigned long mask, unsigned long val)
{
	u32 write_val;
	int ret;

	ret = adin1110_read_reg(priv, reg, &write_val);
	if (ret < 0)
		return ret;

	set_mask_bits(&write_val, mask, val);

	return adin1110_write_reg(priv, reg, write_val);
}

static int adin1110_round_len(int len)
{
	/* can read/write only multiples of 4 bytes of payload */
	len = ALIGN(len, 4);

	if (len + ADIN1110_RD_HEADER_LEN > ADIN1110_MAX_BUFF)
		return -EINVAL;

	return len;
}

static int adin2111_mk_read_fifo(struct adin2111_mk_priv *priv)
{
	u32 header_len = ADIN1110_RD_HEADER_LEN;
	struct spi_transfer t = {0};
	u32 frame_size_no_fcs;
	struct sk_buff *rxb;
	u32 frame_size;
	int round_len;
	u16 reg;
	int ret;

	reg = ADIN1110_RX;
	ret = adin1110_read_reg(priv, ADIN1110_RX_FSIZE, &frame_size);
	if (ret < 0)
		return ret;

	/* The read frame size includes the extra 2 bytes from the ADIN1110 frame header */
	if (frame_size < ADIN1110_FRAME_HEADER_LEN + ADIN1110_FEC_LEN)
		return -EINVAL;

	round_len = adin1110_round_len(frame_size);
	if (round_len < 0)
		return -EINVAL;

	frame_size_no_fcs = frame_size - ADIN1110_FRAME_HEADER_LEN - ADIN1110_FEC_LEN;
	memset(priv->data, 0, ADIN1110_RD_HEADER_LEN);

	priv->data[0] = ADIN1110_CD | FIELD_GET(GENMASK(12, 8), reg);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), reg);

	if (priv->append_crc) {
		priv->data[2] = adin1110_crc_data(&priv->data[0], 2);
		header_len++;
	}

	rxb = netdev_alloc_skb(priv->netdev, round_len + header_len);
	if (!rxb)
		return -ENOMEM;

	skb_put(rxb, frame_size_no_fcs + header_len + ADIN1110_FRAME_HEADER_LEN);

	t.tx_buf = &priv->data[0];
	t.rx_buf = &rxb->data[0];
	t.len = header_len + round_len;

	ret = spi_sync_transfer(priv->spidev, &t, 1);
	if (ret) {
		kfree_skb(rxb);
		return ret;
	}

	skb_pull(rxb, header_len + ADIN1110_FRAME_HEADER_LEN);
	rxb->protocol = eth_type_trans(rxb, priv->netdev);

	netif_rx(rxb);

	priv->rx_bytes += frame_size - ADIN1110_FRAME_HEADER_LEN;
	priv->rx_packets++;

	return 0;
}

static int adin2111_mk_write_fifo(struct adin2111_mk_priv *priv, struct sk_buff *txb)
{
	u32 header_len = ADIN1110_WR_HEADER_LEN;
	__be16 frame_header;
	int padding = 0;
	int padded_len;
	int round_len;
	int ret;

	/* Pad frame to 64 byte length, MAC nor PHY will otherwise add the required padding.
	 * The FEC will be added by the MAC internally.
	 */
	if (txb->len + ADIN1110_FEC_LEN < 64)
		padding = 64 - (txb->len + ADIN1110_FEC_LEN);

	padded_len = txb->len + padding + ADIN1110_FRAME_HEADER_LEN;

	round_len = adin1110_round_len(padded_len);
	if (round_len < 0)
		return round_len;

	ret = adin1110_write_reg(priv, ADIN1110_TX_FSIZE, padded_len);
	if (ret < 0)
		return ret;

	memset(priv->data, 0, round_len + ADIN1110_WR_HEADER_LEN);

	priv->data[0] = ADIN1110_CD | ADIN1110_WRITE;
	priv->data[0] |= FIELD_GET(GENMASK(12, 8), ADIN1110_TX);
	priv->data[1] = FIELD_GET(GENMASK(7, 0), ADIN1110_TX);
	if (priv->append_crc) {
		priv->data[2] = adin1110_crc_data(&priv->data[0], 2);
		header_len++;
	}

	/* Always use port 0 for single interface */
	frame_header = cpu_to_be16(0);
	memcpy(&priv->data[header_len], &frame_header, ADIN1110_FRAME_HEADER_LEN);

	memcpy(&priv->data[header_len + ADIN1110_FRAME_HEADER_LEN], txb->data, txb->len);

	ret = spi_write(priv->spidev, &priv->data[0], round_len + header_len);
	if (ret < 0)
		return ret;

	priv->tx_bytes += txb->len;
	priv->tx_packets++;

	return 0;
}

static int adin1110_read_mdio_acc(struct adin2111_mk_priv *priv)
{
	u32 val;
	int ret;

	mutex_lock(&priv->lock);
	ret = adin1110_read_reg(priv, ADIN1110_MDIOACC, &val);
	mutex_unlock(&priv->lock);
	if (ret < 0)
		return 0;

	return val;
}

static int adin1110_mdio_read(struct mii_bus *bus, int phy_id, int reg)
{
	struct adin2111_mk_priv *priv = bus->priv;
	u32 val = 0;
	int ret;

	if (mdio_phy_id_is_c45(phy_id))
		return -EOPNOTSUPP;

	val |= FIELD_PREP(ADIN1110_MDIO_OP, ADIN1110_MDIO_OP_RD);
	val |= FIELD_PREP(ADIN1110_MDIO_ST, 0x1);
	val |= FIELD_PREP(ADIN1110_MDIO_PRTAD, phy_id);
	val |= FIELD_PREP(ADIN1110_MDIO_DEVAD, reg);

	/* write the clause 22 read command to the chip */
	mutex_lock(&priv->lock);
	ret = adin1110_write_reg(priv, ADIN1110_MDIOACC, val);
	mutex_unlock(&priv->lock);
	if (ret < 0)
		return ret;

	ret = readx_poll_timeout_atomic(adin1110_read_mdio_acc, priv, val,
					(val & ADIN1110_MDIO_TRDONE),
					100, 30000);
	if (ret < 0)
		return ret;

	return (val & ADIN1110_MDIO_DATA);
}

static int adin1110_mdio_write(struct mii_bus *bus, int phy_id, int reg, u16 reg_val)
{
	struct adin2111_mk_priv *priv = bus->priv;
	u32 val = 0;
	int ret;

	if (mdio_phy_id_is_c45(phy_id))
		return -EOPNOTSUPP;

	val |= FIELD_PREP(ADIN1110_MDIO_OP, ADIN1110_MDIO_OP_WR);
	val |= FIELD_PREP(ADIN1110_MDIO_ST, 0x1);
	val |= FIELD_PREP(ADIN1110_MDIO_PRTAD, phy_id);
	val |= FIELD_PREP(ADIN1110_MDIO_DEVAD, reg);
	val |= FIELD_PREP(ADIN1110_MDIO_DATA, reg_val);

	/* write the clause 22 write command to the chip */
	mutex_lock(&priv->lock);
	ret = adin1110_write_reg(priv, ADIN1110_MDIOACC, val);
	mutex_unlock(&priv->lock);
	if (ret < 0)
		return ret;

	return readx_poll_timeout_atomic(adin1110_read_mdio_acc, priv, val,
					 (val & ADIN1110_MDIO_TRDONE),
					 100, 30000);
}

static int adin2111_mk_register_mdiobus(struct adin2111_mk_priv *priv, struct device *dev)
{
	struct mii_bus *mii_bus;
	int ret;

	mii_bus = devm_mdiobus_alloc(dev);
	if (!mii_bus)
		return -ENOMEM;

	snprintf(priv->mii_bus_name, MII_BUS_ID_SIZE, "%s-%u",
		 priv->cfg->name, spi_get_chipselect(priv->spidev, 0));

	mii_bus->name = priv->mii_bus_name;
	mii_bus->read = adin1110_mdio_read;
	mii_bus->write = adin1110_mdio_write;
	mii_bus->priv = priv;
	mii_bus->parent = dev;
	mii_bus->phy_mask = ~((u32)GENMASK(2, 0));
	snprintf(mii_bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));

	ret = devm_mdiobus_register(dev, mii_bus);
	if (ret)
		return ret;

	priv->mii_bus = mii_bus;

	return 0;
}

static void adin2111_mk_read_frames(struct adin2111_mk_priv *priv, unsigned int budget)
{
	u32 status1;
	int ret;

	while (budget) {
		ret = adin1110_read_reg(priv, ADIN1110_STATUS1, &status1);
		if (ret < 0)
			return;

		if (!netif_oper_up(priv->netdev) || !(status1 & ADIN1110_RX_RDY))
			break;

		ret = adin2111_mk_read_fifo(priv);
		if (ret < 0)
			return;

		budget--;
	}
}

static irqreturn_t adin2111_mk_irq(int irq, void *p)
{
	struct adin2111_mk_priv *priv = p;
	u32 status1;
	u32 val;
	int ret;

	mutex_lock(&priv->lock);

	ret = adin1110_read_reg(priv, ADIN1110_STATUS1, &status1);
	if (ret < 0)
		goto out;

	if (priv->append_crc && (status1 & ADIN1110_SPI_ERR))
		dev_warn_ratelimited(&priv->spidev->dev, "SPI CRC error on write.\n");

	ret = adin1110_read_reg(priv, ADIN1110_TX_SPACE, &val);
	if (ret < 0)
		goto out;

	/* TX FIFO space is expressed in half-words */
	priv->tx_space = 2 * val;

	if (netif_oper_up(priv->netdev) && (status1 & ADIN1110_RX_RDY))
		adin2111_mk_read_frames(priv, ADIN1110_MAX_FRAMES_READ);

	/* clear IRQ sources */
	adin1110_write_reg(priv, ADIN1110_STATUS0, ADIN1110_CLEAR_STATUS0);
	adin1110_write_reg(priv, ADIN1110_STATUS1, priv->irq_mask);

out:
	mutex_unlock(&priv->lock);

	if (priv->tx_space > 0 && ret >= 0)
		netif_wake_queue(priv->netdev);

	return IRQ_HANDLED;
}

static int adin2111_mk_write_mac_address(struct adin2111_mk_priv *priv, const u8 *addr)
{
	u32 val;
	int ret;

	/* Use MAC address slot 2 for main interface address */
	val = ADIN1110_MAC_ADDR_APPLY2PORT | ADIN1110_MAC_ADDR_TO_HOST | get_unaligned_be16(&addr[0]);
	ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_UPR + 4, val);
	if (ret < 0)
		return ret;

	val = get_unaligned_be32(&addr[2]);
	return adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_LWR + 4, val);
}

static int adin2111_mk_set_mac_address(struct net_device *netdev, const unsigned char *dev_addr)
{
	struct adin2111_mk_priv *priv = netdev_priv(netdev);

	if (!is_valid_ether_addr(dev_addr))
		return -EADDRNOTAVAIL;

	eth_hw_addr_set(netdev, dev_addr);

	return adin2111_mk_write_mac_address(priv, netdev->dev_addr);
}

static int adin2111_mk_ndo_set_mac_address(struct net_device *netdev, void *addr)
{
	struct sockaddr *sa = addr;
	int ret;

	ret = eth_prepare_mac_addr_change(netdev, addr);
	if (ret < 0)
		return ret;

	return adin2111_mk_set_mac_address(netdev, sa->sa_data);
}

static int adin2111_mk_ioctl(struct net_device *netdev, struct ifreq *rq, int cmd)
{
	if (!netif_running(netdev))
		return -EINVAL;

	return phy_do_ioctl(netdev, rq, cmd);
}

static void adin2111_mk_rx_mode_work(struct work_struct *work)
{
	struct adin2111_mk_priv *priv = container_of(work, struct adin2111_mk_priv, rx_mode_work);

	mutex_lock(&priv->lock);
	/* For single interface, just sync the configuration */
	adin1110_set_bits(priv, ADIN1110_CONFIG1, ADIN1110_CONFIG1_SYNC, ADIN1110_CONFIG1_SYNC);
	mutex_unlock(&priv->lock);
}

static void adin2111_mk_set_rx_mode(struct net_device *dev)
{
	struct adin2111_mk_priv *priv = netdev_priv(dev);

	priv->flags = dev->flags;
	schedule_work(&priv->rx_mode_work);
}

static int adin2111_mk_net_open(struct net_device *net_dev)
{
	struct adin2111_mk_priv *priv = netdev_priv(net_dev);
	u32 val;
	int ret;

	mutex_lock(&priv->lock);

	/* Configure MAC to compute and append the FCS itself */
	ret = adin1110_write_reg(priv, ADIN1110_CONFIG2, ADIN1110_CRC_APPEND);
	if (ret < 0)
		goto out;

	val = ADIN1110_TX_RDY_IRQ | ADIN1110_RX_RDY_IRQ | ADIN1110_SPI_ERR_IRQ;
	priv->irq_mask = val;
	ret = adin1110_write_reg(priv, ADIN1110_IMASK1, ~val);
	if (ret < 0) {
		netdev_err(net_dev, "Failed to enable chip IRQs: %d\n", ret);
		goto out;
	}

	ret = adin1110_read_reg(priv, ADIN1110_TX_SPACE, &val);
	if (ret < 0) {
		netdev_err(net_dev, "Failed to read TX FIFO space: %d\n", ret);
		goto out;
	}

	priv->tx_space = 2 * val;

	ret = adin2111_mk_set_mac_address(net_dev, net_dev->dev_addr);
	if (ret < 0) {
		netdev_err(net_dev, "Could not set MAC address: %pM, %d\n", net_dev->dev_addr, ret);
		goto out;
	}

	ret = adin1110_set_bits(priv, ADIN1110_CONFIG1, ADIN1110_CONFIG1_SYNC, ADIN1110_CONFIG1_SYNC);

out:
	mutex_unlock(&priv->lock);

	if (ret < 0)
		return ret;

	phy_start(priv->phydev);
	netif_start_queue(net_dev);

	return 0;
}

static int adin2111_mk_net_stop(struct net_device *net_dev)
{
	struct adin2111_mk_priv *priv = netdev_priv(net_dev);
	int ret;

	/* Disable RX RDY IRQs */
	mutex_lock(&priv->lock);
	ret = adin1110_set_bits(priv, ADIN1110_IMASK1, ADIN1110_RX_RDY_IRQ, ADIN1110_RX_RDY_IRQ);
	mutex_unlock(&priv->lock);
	if (ret < 0)
		return ret;

	netif_stop_queue(priv->netdev);
	flush_work(&priv->tx_work);
	phy_stop(priv->phydev);

	return 0;
}

static void adin2111_mk_tx_work(struct work_struct *work)
{
	struct adin2111_mk_priv *priv = container_of(work, struct adin2111_mk_priv, tx_work);
	struct sk_buff *txb;
	int ret;

	mutex_lock(&priv->lock);

	while ((txb = skb_dequeue(&priv->txq))) {
		ret = adin2111_mk_write_fifo(priv, txb);
		if (ret < 0)
			dev_err_ratelimited(&priv->spidev->dev, "Frame write error: %d\n", ret);

		dev_kfree_skb(txb);
	}

	mutex_unlock(&priv->lock);
}

static netdev_tx_t adin2111_mk_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct adin2111_mk_priv *priv = netdev_priv(dev);
	netdev_tx_t netdev_ret = NETDEV_TX_OK;
	u32 tx_space_needed;

	tx_space_needed = skb->len + ADIN1110_FRAME_HEADER_LEN + ADIN1110_INTERNAL_SIZE_HEADER_LEN;
	if (tx_space_needed > priv->tx_space) {
		netif_stop_queue(dev);
		netdev_ret = NETDEV_TX_BUSY;
	} else {
		priv->tx_space -= tx_space_needed;
		skb_queue_tail(&priv->txq, skb);
	}

	schedule_work(&priv->tx_work);

	return netdev_ret;
}

static void adin2111_mk_ndo_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *storage)
{
	struct adin2111_mk_priv *priv = netdev_priv(dev);

	storage->rx_packets = priv->rx_packets;
	storage->tx_packets = priv->tx_packets;
	storage->rx_bytes = priv->rx_bytes;
	storage->tx_bytes = priv->tx_bytes;
}

static const struct net_device_ops adin2111_mk_netdev_ops = {
	.ndo_open		= adin2111_mk_net_open,
	.ndo_stop		= adin2111_mk_net_stop,
	.ndo_eth_ioctl		= adin2111_mk_ioctl,
	.ndo_start_xmit		= adin2111_mk_start_xmit,
	.ndo_set_mac_address	= adin2111_mk_ndo_set_mac_address,
	.ndo_set_rx_mode	= adin2111_mk_set_rx_mode,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_get_stats64	= adin2111_mk_ndo_get_stats64,
};

static void adin2111_mk_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *di)
{
	strscpy(di->driver, "ADIN2111-MK", sizeof(di->driver));
	strscpy(di->bus_info, dev_name(dev->dev.parent), sizeof(di->bus_info));
}

static const struct ethtool_ops adin2111_mk_ethtool_ops = {
	.get_drvinfo		= adin2111_mk_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
};

static void adin2111_mk_adjust_link(struct net_device *dev)
{
	struct phy_device *phydev = dev->phydev;

	if (!phydev->link)
		phy_print_status(phydev);
}

/* PHY ID is stored in the MAC registers too, check spi connection by reading it */
static int adin2111_mk_check_spi(struct adin2111_mk_priv *priv)
{
	struct gpio_desc *reset_gpio;
	int ret;
	u32 val;

	reset_gpio = devm_gpiod_get_optional(&priv->spidev->dev, "reset", GPIOD_OUT_LOW);
	if (reset_gpio) {
		/* MISO pin is used for internal configuration, can't have anyone else disturbing the SDO line */
		spi_bus_lock(priv->spidev->controller);

		gpiod_set_value(reset_gpio, 1);
		fsleep(10000);
		gpiod_set_value(reset_gpio, 0);

		/* Need to wait 90 ms before interacting with the MAC after a HW reset */
		fsleep(90000);

		spi_bus_unlock(priv->spidev->controller);
	}

	ret = adin1110_read_reg(priv, ADIN1110_PHY_ID, &val);
	if (ret < 0)
		return ret;

	if (val != priv->cfg->phy_id_val) {
		dev_err(&priv->spidev->dev, "PHY ID expected: %x, read: %x\n", priv->cfg->phy_id_val, val);
		return -EIO;
	}

	return 0;
}

static void adin2111_mk_disconnect_phy(void *data)
{
	phy_disconnect(data);
}

static int adin2111_mk_probe_netdev(struct adin2111_mk_priv *priv)
{
	struct device *dev = &priv->spidev->dev;
	struct net_device *netdev;
	int ret;

	netdev = devm_alloc_etherdev(dev, 0);
	if (!netdev)
		return -ENOMEM;

	/* Set up the network device */
	priv->netdev = netdev;
	SET_NETDEV_DEV(netdev, dev);
	netdev_assign_priv(netdev, priv);

	ret = device_get_ethdev_address(dev, netdev);
	if (ret < 0)
		return ret;

	netdev->irq = priv->spidev->irq;
	INIT_WORK(&priv->tx_work, adin2111_mk_tx_work);
	INIT_WORK(&priv->rx_mode_work, adin2111_mk_rx_mode_work);
	skb_queue_head_init(&priv->txq);

	netif_carrier_off(netdev);

	netdev->if_port = IF_PORT_10BASET;
	netdev->netdev_ops = &adin2111_mk_netdev_ops;
	netdev->ethtool_ops = &adin2111_mk_ethtool_ops;
	netdev->priv_flags |= IFF_UNICAST_FLT;

	/* Get and connect PHY */
	priv->phydev = get_phy_device(priv->mii_bus, priv->cfg->phy_id, false);
	if (IS_ERR(priv->phydev)) {
		netdev_err(netdev, "Could not find PHY with device address: %d.\n", priv->cfg->phy_id);
		return PTR_ERR(priv->phydev);
	}

	priv->phydev = phy_connect(netdev, phydev_name(priv->phydev), adin2111_mk_adjust_link, PHY_INTERFACE_MODE_INTERNAL);
	if (IS_ERR(priv->phydev)) {
		netdev_err(netdev, "Could not connect PHY with device address: %d.\n", priv->cfg->phy_id);
		return PTR_ERR(priv->phydev);
	}

	ret = devm_add_action_or_reset(dev, adin2111_mk_disconnect_phy, priv->phydev);
	if (ret < 0)
		return ret;

	/* Request IRQ */
	ret = devm_request_threaded_irq(dev, priv->spidev->irq, NULL, adin2111_mk_irq,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT, dev_name(dev), priv);
	if (ret < 0)
		return ret;

	/* Register network device */
	ret = devm_register_netdev(dev, netdev);
	if (ret < 0) {
		dev_err(dev, "Failed to register network device.\n");
		return ret;
	}

	return 0;
}

static int adin2111_mk_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct adin2111_mk_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(struct adin2111_mk_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	priv->spidev = spi;
	priv->cfg = &adin2111_mk_cfg;
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;

	mutex_init(&priv->lock);

	/* use of CRC on control and data transactions is pin dependent */
	priv->append_crc = device_property_read_bool(dev, "adi,spi-crc");
	if (priv->append_crc)
		crc8_populate_msb(adin1110_crc_table, 0x7);

	ret = adin2111_mk_check_spi(priv);
	if (ret < 0) {
		dev_err(dev, "Probe SPI Read check failed: %d\n", ret);
		return ret;
	}

	ret = adin1110_write_reg(priv, ADIN1110_RESET, ADIN1110_SWRESET);
	if (ret < 0)
		return ret;

	ret = adin2111_mk_register_mdiobus(priv, dev);
	if (ret < 0) {
		dev_err(dev, "Could not register MDIO bus %d\n", ret);
		return ret;
	}

	return adin2111_mk_probe_netdev(priv);
}

static const struct of_device_id adin2111_mk_match_table[] = {
	{ .compatible = "adi,adin2111-mk" },
	{ }
};
MODULE_DEVICE_TABLE(of, adin2111_mk_match_table);

static const struct spi_device_id adin2111_mk_spi_id[] = {
	{ .name = "adin2111-mk", .driver_data = ADIN2111_MAC_MK },
	{ }
};
MODULE_DEVICE_TABLE(spi, adin2111_mk_spi_id);

static struct spi_driver adin2111_mk_driver = {
	.driver = {
		.name = "adin2111-mk",
		.of_match_table = adin2111_mk_match_table,
	},
	.probe = adin2111_mk_probe,
	.id_table = adin2111_mk_spi_id,
};

module_spi_driver(adin2111_mk_driver);

MODULE_DESCRIPTION("ADIN2111-MK Single Interface Network driver");
MODULE_AUTHOR("Murray Kopit <murray.kopit@analog.com>");
MODULE_LICENSE("Dual BSD/GPL");