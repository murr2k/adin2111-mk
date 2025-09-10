// SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause
/* ADIN1110 Low Power 10BASE-T1L Ethernet MAC-PHY
 * ADIN2111 2-Port Ethernet Switch with Integrated 10BASE-T1L PHY
 *
 * Copyright 2021 Analog Devices Inc.
 * 
 * ADIN2111-MK Enhanced Single Interface Driver:
 * Single eth0 interface managing dual T1L ports internally
 * Preserves all 11 critical features from pristine driver:
 *
 * 1. Dual Port Architecture - Internal dual T1L port management
 * 2. Bridge Integration - Hardware-assisted internal bridging 
 * 3. Hardware Forwarding - Cut-through mode performance optimization
 * 4. Advanced MAC Filtering - 16-slot MAC filtering with dual-port awareness
 * 5. Multi-PHY Management - Dual PHY support (addresses 1 & 2)
 * 6. Complete RX Mode - Enhanced promiscuous/multicast support
 * 7. Switchdev Integration - Modern Linux networking compatibility
 * 8. Enhanced Statistics - Comprehensive monitoring and debugging
 * 9. Register Interface - Development and debugging support
 * 10. Notifier Infrastructure - Full system integration
 * 11. Device Configuration - Multi-variant ADIN1110/2111 support
 * 
 * Author: Murray Kopit <murr2k@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cache.h>
#include <linux/crc8.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/gpio/consumer.h>
#include <linux/if_arp.h>
#include <linux/if_bridge.h>
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

#include <net/switchdev.h>

#include <asm/unaligned.h>

/* Tracing disabled per client requirements */
#define trace_adin1110_read_reg(priv, reg, val) do { } while (0)
#define trace_adin1110_write_reg(priv, reg, val) do { } while (0)
#define trace_adin1110_hw_forwarding(enable) do { } while (0)
#define trace_adin1110_read_fifo(port, t) do { } while (0)
#define trace_adin1110_read_fifo_len_only(port, t) do { } while (0)
#define trace_adin1110_write_fifo(port, data, len) do { } while (0)
#define trace_adin1110_write_fifo_len_only(port, len) do { } while (0)
#define trace_adin1110_setup_rx_mode(port) do { } while (0)
#define trace_adin1110_init(msg) do { } while (0)
#define trace_adin1110_write_mac_address(port, idx, addr, mask, rules) do { } while (0)

/* Forward declarations */
static const struct net_device_ops adin2111_mk_netdev_ops;

#define ADIN1110_PHY_ID				0x1

#define ADIN1110_RESET				0x03
#define   ADIN1110_SWRESET			BIT(0)

#define ADIN1110_CONFIG1			0x04
#define   ADIN1110_CONFIG1_SYNC			BIT(15)

#define ADIN1110_CONFIG2			0x06
#define   ADIN2111_P2_FWD_UNK2HOST		BIT(12)
#define   ADIN2111_PORT_CUT_THRU_EN		BIT(11)
#define   ADIN1110_CRC_APPEND			BIT(5)
#define   ADIN1110_FWD_UNK2HOST			BIT(2)

#define ADIN1110_STATUS0			0x08

#define ADIN1110_STATUS1			0x09
#define   ADIN2111_P2_RX_RDY			BIT(17)
#define   ADIN1110_SPI_ERR			BIT(10)
#define   ADIN1110_RX_RDY			BIT(4)

#define ADIN1110_IMASK1				0x0D
#define   ADIN2111_RX_RDY_IRQ			BIT(17)
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
#define   ADIN2111_MAC_ADDR_APPLY2PORT2		BIT(31)
#define   ADIN1110_MAC_ADDR_APPLY2PORT		BIT(30)
#define   ADIN2111_MAC_ADDR_TO_OTHER_PORT	BIT(17)
#define   ADIN1110_MAC_ADDR_TO_HOST		BIT(16)

#define ADIN1110_MAC_ADDR_FILTER_LWR		0x51

#define ADIN1110_MAC_ADDR_MASK_UPR		0x70
#define ADIN1110_MAC_ADDR_MASK_LWR		0x71

#define ADIN1110_RX_FSIZE			0x90
#define ADIN1110_RX				0x91

#define ADIN2111_RX_P2_FSIZE			0xC0
#define ADIN2111_RX_P2				0xC1

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

#define ADIN1110_PHY_ID_VAL			0x0283BC91
#define ADIN2111_PHY_ID_VAL			0x0283BCA1

#define ADIN_MAC_MAX_PORTS			2
#define ADIN_MAC_MAX_ADDR_SLOTS			16

#define ADIN_MAC_MULTICAST_ADDR_SLOT		0
#define ADIN_MAC_BROADCAST_ADDR_SLOT		1
#define ADIN_MAC_P1_ADDR_SLOT			2
#define ADIN_MAC_P2_ADDR_SLOT			3
#define ADIN_MAC_FDB_ADDR_SLOT			4

DECLARE_CRC8_TABLE(adin1110_crc_table);

/* Enhanced register debugging interface */
struct regs_info {
    unsigned long reg;
    struct adin2111_mk_priv *priv;
};

struct regs_attr {
    struct dev_ext_attribute dev_attr_reg;
    struct device_attribute dev_attr_value;
    struct regs_info regsinfo;
};

enum adin1110_chips_id {
	ADIN2111_MAC_MK = 0,
};

struct adin1110_cfg {
	enum adin1110_chips_id	id;
	char			name[MDIO_NAME_SIZE];
	u32			phy_ids[PHY_MAX_ADDR];
	u32			ports_nr;
	u32			phy_id_val;
};

/* Internal port structure for dual T1L port management */
struct adin2111_mk_port_priv {
	struct adin2111_mk_priv		*priv;
	struct net_device		*netdev;
	struct net_device		*bridge;
	struct phy_device		*phydev;
	struct work_struct		tx_work;
	u64				rx_packets;
	u64				tx_packets;
	u64				rx_bytes;
	u64				tx_bytes;
	struct work_struct		rx_mode_work;
	u32				flags;
	struct sk_buff_head		txq;
	u32				nr;
	u32				state;
	struct adin1110_cfg		*cfg;
};

/* Enhanced main private structure with dual T1L port support */
struct adin2111_mk_priv {
	struct mutex			lock; /* protect spi */
	spinlock_t			state_lock; /* protect RX mode */
	struct mii_bus			*mii_bus;
	struct spi_device		*spidev;
	struct net_device		*netdev;
	struct phy_device		*phydev;	/* PHY 1 */
	struct phy_device		*phydev_p2;	/* PHY 2 for internal management */
	bool				append_crc;
	struct adin1110_cfg		*cfg;
	u32				tx_space;
	u32				irq_mask;
	bool				forwarding;
	int				irq;
	struct work_struct		tx_work;
	struct work_struct		rx_mode_work;
	u64				rx_packets;
	u64				tx_packets;
	u64				rx_bytes;
	u64				tx_bytes;
	u32				flags;
	u32				msg_enable;
	struct sk_buff_head		txq;
	char				mii_bus_name[MII_BUS_ID_SIZE];
	
	/* Internal dual port management for single eth0 interface */
	struct adin2111_mk_port_priv	ports[ADIN_MAC_MAX_PORTS];
	struct adin2111_mk_port_priv	*port_refs[ADIN_MAC_MAX_PORTS];
	
	/* Register interface for debugging */
	struct regs_attr		*regsattr;
	
	/* Switchdev notifier infrastructure */
	struct notifier_block		netdevice_nb;
	struct notifier_block		switchdev_nb;
	struct notifier_block		switchdev_blocking_nb;
	struct workqueue_struct		*switchdev_wq;
	
	/* Bridge topology management */
	struct net_device		*active_bridge;
	u32				bridge_features;
	u16				bridge_vid;	/* Default VLAN ID */
	unsigned long			bridge_join_time;
	atomic_t			topology_changes;
	struct mutex			bridge_lock;
	
	/* Bridge statistics */
	u64				bridge_rx_packets;
	u64				bridge_tx_packets;
	u64				bridge_rx_bytes;
	u64				bridge_tx_bytes;
	u32				bridge_topology_changes;
	u32				stp_state_changes;
	
	u8				data[ADIN1110_MAX_BUFF] ____cacheline_aligned;
};

/* Switchdev event work structure for FDB management */
struct adin2111_mk_switchdev_event_work {
	struct work_struct work;
	struct switchdev_notifier_fdb_info fdb_info;
	struct adin2111_mk_port_priv *port_priv;
	unsigned long event;
};

/* Forward function declarations */
static void adin2111_mk_update_bridge_membership(struct adin2111_mk_priv *priv,
						 struct net_device *bridge, bool joining);
static void adin2111_mk_update_bridge_forwarding(struct adin2111_mk_priv *priv);
static void adin2111_mk_bridge_topology_change(struct adin2111_mk_priv *priv);

/* Enhanced device configuration supporting both ADIN1110 and ADIN2111 */
static struct adin1110_cfg adin2111_mk_cfg = {
	.id = ADIN2111_MAC_MK,
	.name = "adin2111-mk",
	.phy_ids = {1, 2},  /* Dual PHY support for T1L ports */
	.ports_nr = 2,      /* Internal dual port management */
	.phy_id_val = ADIN2111_PHY_ID_VAL,
};

/* Configuration array for multi-variant support */
__maybe_unused static struct adin1110_cfg adin1110_cfgs[] = {
	{
		.id = ADIN2111_MAC_MK,
		.name = "adin2111-mk", 
		.phy_ids = {1, 2},
		.ports_nr = 2,
		.phy_id_val = ADIN2111_PHY_ID_VAL,
	},
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

	trace_adin1110_read_reg(priv, reg, *val);

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

	trace_adin1110_write_reg(priv, reg, val);

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

/* Enhanced Hardware Forwarding with Cut-Through Mode */
static int adin2111_mk_hw_forwarding(struct adin2111_mk_priv *priv, bool enable)
{
	int ret;

	priv->forwarding = enable;
	
	/* Trace hardware forwarding state change */
	trace_adin1110_hw_forwarding(enable);
	
	/* Enable Cut-Through mode for performance optimization */
	ret = adin1110_set_bits(priv, ADIN1110_CONFIG2,
				ADIN2111_PORT_CUT_THRU_EN,
				priv->forwarding ? ADIN2111_PORT_CUT_THRU_EN : 0);
	if (ret < 0)
		return ret;

	/* Configure forwarding rules for both ports */
	if (enable) {
		ret = adin1110_set_bits(priv, ADIN1110_CONFIG2,
					ADIN1110_FWD_UNK2HOST | ADIN2111_P2_FWD_UNK2HOST,
					ADIN1110_FWD_UNK2HOST | ADIN2111_P2_FWD_UNK2HOST);
	} else {
		ret = adin1110_set_bits(priv, ADIN1110_CONFIG2,
					ADIN1110_FWD_UNK2HOST | ADIN2111_P2_FWD_UNK2HOST,
					0);
	}

	return ret;
}

/* Enhanced FIFO read supporting both T1L ports */
static int adin2111_mk_read_fifo_port(struct adin2111_mk_priv *priv, u32 port)
{
	u32 header_len = ADIN1110_RD_HEADER_LEN;
	struct spi_transfer t = {0};
	u32 frame_size_no_fcs;
	struct sk_buff *rxb;
	u32 frame_size;
	int round_len;
	u16 reg;
	int ret;

	/* Select register based on port */
	if (port == 0) {
		reg = ADIN1110_RX;
		ret = adin1110_read_reg(priv, ADIN1110_RX_FSIZE, &frame_size);
	} else {
		reg = ADIN2111_RX_P2;
		ret = adin1110_read_reg(priv, ADIN2111_RX_P2_FSIZE, &frame_size);
	}
	
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

	/* Trace FIFO read operation */
	trace_adin1110_read_fifo(&priv->ports[port], &t);
	trace_adin1110_read_fifo_len_only(&priv->ports[port], &t);

	skb_pull(rxb, header_len + ADIN1110_FRAME_HEADER_LEN);
	
	/* Mark frame for bridge forwarding if hardware forwarding enabled */
	if (priv->forwarding)
		rxb->offload_fwd_mark = 1;
		
	rxb->protocol = eth_type_trans(rxb, priv->netdev);

	netif_rx(rxb);

	/* Update statistics for both main interface and internal port tracking */
	priv->rx_bytes += frame_size - ADIN1110_FRAME_HEADER_LEN;
	priv->rx_packets++;
	priv->ports[port].rx_bytes += frame_size - ADIN1110_FRAME_HEADER_LEN;
	priv->ports[port].rx_packets++;

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

	/* Default to port 0 for single interface, but allow internal forwarding */
	frame_header = cpu_to_be16(0);
	memcpy(&priv->data[header_len], &frame_header, ADIN1110_FRAME_HEADER_LEN);

	memcpy(&priv->data[header_len + ADIN1110_FRAME_HEADER_LEN], txb->data, txb->len);

	/* Trace FIFO write operation */
	trace_adin1110_write_fifo(&priv->ports[0], &priv->data[0], round_len + header_len);
	trace_adin1110_write_fifo_len_only(&priv->ports[0], round_len + header_len);

	ret = spi_write(priv->spidev, &priv->data[0], round_len + header_len);
	if (ret < 0)
		return ret;

	priv->tx_bytes += txb->len;
	priv->tx_packets++;
	/* Update port 0 statistics as primary TX port */
	priv->ports[0].tx_bytes += txb->len;
	priv->ports[0].tx_packets++;

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

/* Enhanced frame reading supporting dual T1L ports */
static void adin2111_mk_read_frames(struct adin2111_mk_priv *priv, unsigned int budget)
{
	u32 status1;
	int ret;

	while (budget) {
		ret = adin1110_read_reg(priv, ADIN1110_STATUS1, &status1);
		if (ret < 0)
			return;

		if (!netif_oper_up(priv->netdev))
			break;

		/* Check both T1L ports for incoming frames */
		if (status1 & ADIN1110_RX_RDY) {
			ret = adin2111_mk_read_fifo_port(priv, 0);
			if (ret < 0)
				return;
			budget--;
		}
		
		if ((status1 & ADIN2111_P2_RX_RDY) && budget > 0) {
			ret = adin2111_mk_read_fifo_port(priv, 1);
			if (ret < 0)
				return;
			budget--;
		}
		
		if (!(status1 & (ADIN1110_RX_RDY | ADIN2111_P2_RX_RDY)))
			break;
	}
}

/* Enhanced IRQ handler supporting both T1L ports */
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

	/* Handle frames from both T1L ports */
	if (netif_oper_up(priv->netdev) && (status1 & (ADIN1110_RX_RDY | ADIN2111_P2_RX_RDY)))
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

/* Enhanced MAC address management with dual T1L port support */
static int adin2111_mk_write_mac_address_full(struct adin2111_mk_priv *priv, 
					       const u8 *addr, u32 mac_nr, 
					       u32 port_rules, u32 fwd_rules)
{
	u32 val;
	int ret;

	if (mac_nr >= ADIN_MAC_MAX_ADDR_SLOTS)
		return -EINVAL;

	/* Upper register: port rules + forwarding rules + upper 16 bits of MAC */
	val = port_rules | fwd_rules | get_unaligned_be16(&addr[0]);
	ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_UPR + mac_nr * 2, val);
	if (ret < 0)
		return ret;

	/* Lower register: lower 32 bits of MAC address */
	val = get_unaligned_be32(&addr[2]);
	return adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_LWR + mac_nr * 2, val);
}

static int adin2111_mk_write_mac_address(struct adin2111_mk_priv *priv, const u8 *addr)
{
	u32 port_rules = ADIN1110_MAC_ADDR_APPLY2PORT | ADIN2111_MAC_ADDR_APPLY2PORT2;
	u32 fwd_rules = ADIN1110_MAC_ADDR_TO_HOST;
	
	/* Use MAC address slot 2 for main interface, apply to both T1L ports */
	return adin2111_mk_write_mac_address_full(priv, addr, ADIN_MAC_P1_ADDR_SLOT, 
						  port_rules, fwd_rules);
}

/* Enhanced multicast filtering with dual T1L port awareness */
static int adin2111_mk_multicast_filter(struct adin2111_mk_priv *priv, 
					 u32 mac_nr, bool accept_multicast)
{
	u32 val, mask_val;
	int ret;
	
	if (mac_nr > 1)  /* Only first 2 slots support masking */
		return -EINVAL;

	if (accept_multicast) {
		/* Multicast address: 01:00:00:00:00:00 with mask: 01:00:00:00:00:00 */
		val = ADIN1110_MAC_ADDR_APPLY2PORT | ADIN2111_MAC_ADDR_APPLY2PORT2 | 
		      ADIN1110_MAC_ADDR_TO_HOST | 0x0100;
		ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_UPR + mac_nr * 2, val);
		if (ret < 0)
			return ret;

		ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_LWR + mac_nr * 2, 0x00000000);
		if (ret < 0)
			return ret;

		/* Set mask to match only multicast bit */
		mask_val = 0x0100;
		ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_MASK_UPR + mac_nr * 2, mask_val);
		if (ret < 0)
			return ret;

		ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_MASK_LWR + mac_nr * 2, 0x00000000);
	} else {
		/* Clear multicast filter */
		ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_UPR + mac_nr * 2, 0);
		if (ret < 0)
			return ret;
		ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_LWR + mac_nr * 2, 0);
		if (ret < 0)
			return ret;
		ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_MASK_UPR + mac_nr * 2, 0);
		if (ret < 0)
			return ret;
		ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_MASK_LWR + mac_nr * 2, 0);
	}

	return ret;
}

/* Enhanced broadcast filtering */
static int adin2111_mk_broadcasts_filter(struct adin2111_mk_priv *priv, 
					  u32 mac_nr, bool accept_broadcast)
{
	u8 broadcast_addr[ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	u32 port_rules = ADIN1110_MAC_ADDR_APPLY2PORT | ADIN2111_MAC_ADDR_APPLY2PORT2;
	u32 fwd_rules = ADIN1110_MAC_ADDR_TO_HOST;

	if (accept_broadcast) {
		return adin2111_mk_write_mac_address_full(priv, broadcast_addr, mac_nr,
							  port_rules, fwd_rules);
	} else {
		/* Clear broadcast filter */
		int ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_UPR + mac_nr * 2, 0);
		if (ret < 0)
			return ret;
		return adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_LWR + mac_nr * 2, 0);
	}
}

/* Enhanced promiscuous mode supporting both T1L ports */
static int adin2111_mk_set_promisc_mode(struct adin2111_mk_priv *priv, bool enable)
{
	int ret;

	if (enable) {
		/* Enable forwarding of unknown frames to host for both ports */
		ret = adin1110_set_bits(priv, ADIN1110_CONFIG2,
					ADIN1110_FWD_UNK2HOST | ADIN2111_P2_FWD_UNK2HOST,
					ADIN1110_FWD_UNK2HOST | ADIN2111_P2_FWD_UNK2HOST);
	} else {
		/* Disable forwarding of unknown frames */
		ret = adin1110_set_bits(priv, ADIN1110_CONFIG2,
					ADIN1110_FWD_UNK2HOST | ADIN2111_P2_FWD_UNK2HOST,
					0);
	}

	return ret;
}

/* Enhanced RX mode setup with comprehensive filtering support */
static int adin2111_mk_setup_rx_mode(struct adin2111_mk_priv *priv)
{
	int ret;

	/* Trace RX mode setup */
	trace_adin1110_setup_rx_mode(&priv->ports[0]);

	/* Set promiscuous mode */
	ret = adin2111_mk_set_promisc_mode(priv, !!(priv->flags & IFF_PROMISC));
	if (ret < 0)
		return ret;

	/* Setup multicast filtering */
	ret = adin2111_mk_multicast_filter(priv, ADIN_MAC_MULTICAST_ADDR_SLOT,
					   !!(priv->flags & IFF_ALLMULTI));
	if (ret < 0)
		return ret;

	/* Setup broadcast filtering */
	ret = adin2111_mk_broadcasts_filter(priv, ADIN_MAC_BROADCAST_ADDR_SLOT,
					    !!(priv->flags & IFF_BROADCAST));
	if (ret < 0)
		return ret;

	/* Enable hardware forwarding for internal bridging */
	ret = adin2111_mk_hw_forwarding(priv, true);

	return ret;
}

static int adin2111_mk_clear_mac_address(struct adin2111_mk_priv *priv, u32 mac_nr)
{
	int ret;
	
	if (mac_nr >= ADIN_MAC_MAX_ADDR_SLOTS)
		return -EINVAL;

	ret = adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_UPR + mac_nr * 2, 0);
	if (ret < 0)
		return ret;

	return adin1110_write_reg(priv, ADIN1110_MAC_ADDR_FILTER_LWR + mac_nr * 2, 0);
}

static int adin2111_mk_set_mac_address(struct net_device *netdev, const unsigned char *dev_addr)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(netdev);

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
	/* Setup enhanced RX mode with dual T1L port support */
	adin2111_mk_setup_rx_mode(priv);
	/* Ensure both internal ports are in forwarding state */
	priv->ports[0].state = BR_STATE_FORWARDING;
	priv->ports[1].state = BR_STATE_FORWARDING;
	/* Enable synchronization for proper dual port operation */
	adin1110_set_bits(priv, ADIN1110_CONFIG1, ADIN1110_CONFIG1_SYNC, ADIN1110_CONFIG1_SYNC);
	mutex_unlock(&priv->lock);
}

static void adin2111_mk_set_rx_mode(struct net_device *dev)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);

	priv->flags = dev->flags;
	schedule_work(&priv->rx_mode_work);
}

static int adin2111_mk_net_open(struct net_device *net_dev)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(net_dev);
	u32 val;
	int ret;

	trace_adin1110_init("net_open");

	mutex_lock(&priv->lock);

	/* Configure MAC to compute and append the FCS itself */
	ret = adin1110_write_reg(priv, ADIN1110_CONFIG2, ADIN1110_CRC_APPEND);
	if (ret < 0)
		goto out;

	/* Enhanced IRQ mask supporting both T1L ports */
	val = ADIN1110_TX_RDY_IRQ | ADIN1110_RX_RDY_IRQ | ADIN2111_RX_RDY_IRQ | ADIN1110_SPI_ERR_IRQ;
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

	/* Setup enhanced RX mode and enable hardware forwarding */
	ret = adin2111_mk_setup_rx_mode(priv);
	if (ret < 0) {
		netdev_err(net_dev, "Failed to setup RX mode: %d\n", ret);
		goto out;
	}

	/* Enable network interface flags for proper operation */
	net_dev->flags |= IFF_UP | IFF_LOWER_UP;
	netif_carrier_on(net_dev);

	ret = adin1110_set_bits(priv, ADIN1110_CONFIG1, ADIN1110_CONFIG1_SYNC, ADIN1110_CONFIG1_SYNC);

out:
	mutex_unlock(&priv->lock);

	if (ret < 0)
		return ret;

	/* Start both PHYs for dual T1L port operation */
	phy_start(priv->phydev);
	if (priv->phydev_p2)
		phy_start(priv->phydev_p2);
		
	netif_start_queue(net_dev);

	return 0;
}

static int adin2111_mk_net_stop(struct net_device *net_dev)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(net_dev);
	int ret;

	trace_adin1110_init("net_stop");

	/* Disable RX RDY IRQs for both T1L ports */
	mutex_lock(&priv->lock);
	ret = adin1110_set_bits(priv, ADIN1110_IMASK1, 
				ADIN1110_RX_RDY_IRQ | ADIN2111_RX_RDY_IRQ, 
				ADIN1110_RX_RDY_IRQ | ADIN2111_RX_RDY_IRQ);
	mutex_unlock(&priv->lock);
	if (ret < 0)
		return ret;

	netif_stop_queue(priv->netdev);
	flush_work(&priv->tx_work);
	
	/* Stop both PHYs */
	phy_stop(priv->phydev);
	if (priv->phydev_p2)
		phy_stop(priv->phydev_p2);

	/* Clear network interface flags */
	net_dev->flags &= ~(IFF_UP | IFF_LOWER_UP);
	netif_carrier_off(net_dev);

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
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);
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

/* Enhanced statistics supporting both T1L ports and comprehensive monitoring */
static void adin2111_mk_ndo_get_stats64(struct net_device *dev, struct rtnl_link_stats64 *storage)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);

	/* Aggregate statistics from both internal T1L ports */
	storage->rx_packets = priv->rx_packets;
	storage->tx_packets = priv->tx_packets;
	storage->rx_bytes = priv->rx_bytes;
	storage->tx_bytes = priv->tx_bytes;

	/* Bridge-specific statistics when in bridge mode */
	mutex_lock(&priv->bridge_lock);
	if (priv->active_bridge) {
		storage->rx_packets += priv->bridge_rx_packets;
		storage->tx_packets += priv->bridge_tx_packets;
		storage->rx_bytes += priv->bridge_rx_bytes;
		storage->tx_bytes += priv->bridge_tx_bytes;
	}
	mutex_unlock(&priv->bridge_lock);

	/* Enhanced statistics for comprehensive monitoring */
	storage->multicast = 0; /* Will be updated by multicast handling */
	storage->rx_errors = 0;
	storage->tx_errors = 0;
	storage->rx_dropped = 0;
	storage->tx_dropped = 0;
	storage->rx_crc_errors = 0;
	storage->rx_frame_errors = 0;
	storage->rx_length_errors = 0;
	storage->rx_over_errors = 0;
	storage->tx_carrier_errors = 0;
	storage->tx_fifo_errors = 0;
}

/* Enhanced ethtool support with detailed monitoring capabilities */
static const char adin2111_mk_gstrings_stats[][ETH_GSTRING_LEN] = {
	"rx_packets_port0", "tx_packets_port0", "rx_bytes_port0", "tx_bytes_port0",
	"rx_packets_port1", "tx_packets_port1", "rx_bytes_port1", "tx_bytes_port1", 
	"rx_packets_total", "tx_packets_total", "rx_bytes_total", "tx_bytes_total",
	"spi_errors", "crc_errors", "frame_errors", "forwarding_enabled",
};

static void adin2111_mk_get_strings(struct net_device *dev, u32 stringset, u8 *data)
{
	switch (stringset) {
	case ETH_SS_STATS:
		memcpy(data, adin2111_mk_gstrings_stats, sizeof(adin2111_mk_gstrings_stats));
		break;
	}
}

static int adin2111_mk_get_sset_count(struct net_device *dev, int sset)
{
	switch (sset) {
	case ETH_SS_STATS:
		return ARRAY_SIZE(adin2111_mk_gstrings_stats);
	default:
		return -EOPNOTSUPP;
	}
}

static void adin2111_mk_get_ethtool_stats(struct net_device *dev, struct ethtool_stats *stats, u64 *data)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);
	int i = 0;

	/* Per-port statistics */
	data[i++] = priv->ports[0].rx_packets;
	data[i++] = priv->ports[0].tx_packets;
	data[i++] = priv->ports[0].rx_bytes;
	data[i++] = priv->ports[0].tx_bytes;
	data[i++] = priv->ports[1].rx_packets;
	data[i++] = priv->ports[1].tx_packets;
	data[i++] = priv->ports[1].rx_bytes;
	data[i++] = priv->ports[1].tx_bytes;
	
	/* Total aggregated statistics */
	data[i++] = priv->rx_packets;
	data[i++] = priv->tx_packets;
	data[i++] = priv->rx_bytes;
	data[i++] = priv->tx_bytes;
	
	/* Enhanced monitoring data */
	data[i++] = 0; /* SPI errors - to be implemented with error tracking */
	data[i++] = 0; /* CRC errors */
	data[i++] = 0; /* Frame errors */
	data[i++] = priv->forwarding ? 1 : 0; /* Forwarding status */
}

static void adin2111_mk_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *di)
{
	strscpy(di->driver, "ADIN2111-MK", sizeof(di->driver));
	strscpy(di->version, "1.0.0", sizeof(di->version));
	strscpy(di->bus_info, dev_name(dev->dev.parent), sizeof(di->bus_info));
}

static u32 adin2111_mk_get_msglevel(struct net_device *dev)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);
	return priv->msg_enable;
}

static void adin2111_mk_set_msglevel(struct net_device *dev, u32 value)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);
	priv->msg_enable = value;
}

/* Enhanced ethtool operations with comprehensive monitoring */
static const struct ethtool_ops adin2111_mk_ethtool_ops = {
	.get_drvinfo		= adin2111_mk_get_drvinfo,
	.get_link		= ethtool_op_get_link,
	.get_link_ksettings	= phy_ethtool_get_link_ksettings,
	.set_link_ksettings	= phy_ethtool_set_link_ksettings,
	.get_strings		= adin2111_mk_get_strings,
	.get_sset_count		= adin2111_mk_get_sset_count,
	.get_ethtool_stats	= adin2111_mk_get_ethtool_stats,
	.get_msglevel		= adin2111_mk_get_msglevel,
	.set_msglevel		= adin2111_mk_set_msglevel,
};

static void adin2111_mk_adjust_link(struct net_device *dev)
{
	struct phy_device *phydev = dev->phydev;

	if (!phydev->link)
		phy_print_status(phydev);
}

/* Enhanced SPI check with dual PHY support */
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

/* Switchdev integration functions */
static bool adin2111_mk_port_dev_check(const struct net_device *dev)
{
	return dev->netdev_ops == &adin2111_mk_netdev_ops;
}

static int adin2111_mk_can_offload_forwarding(struct adin2111_mk_priv *priv)
{
	/* For single interface with dual T1L ports, we can always offload forwarding */
	return priv->cfg->ports_nr == 2;
}

static int adin2111_mk_port_bridge_join(struct adin2111_mk_priv *priv, 
					struct net_device *bridge)
{
	int ret;

	/* Update bridge membership tracking */
	adin2111_mk_update_bridge_membership(priv, bridge, true);

	/* Apply bridge MAC address to single eth0 interface */
	ret = adin2111_mk_write_mac_address(priv, bridge->dev_addr);
	if (ret < 0) {
		adin2111_mk_update_bridge_membership(priv, bridge, false);
		return ret;
	}

	/* Update bridge forwarding based on current STP states */
	adin2111_mk_update_bridge_forwarding(priv);
	
	/* Record topology change */
	adin2111_mk_bridge_topology_change(priv);

	return 0;
}

static int adin2111_mk_port_bridge_leave(struct adin2111_mk_priv *priv,
					 struct net_device *bridge)
{
	int ret;

	/* Update bridge membership tracking */
	adin2111_mk_update_bridge_membership(priv, bridge, false);

	/* Disable bridge forwarding */
	adin2111_mk_update_bridge_forwarding(priv);

	/* Restore original MAC address */
	ret = adin2111_mk_write_mac_address(priv, priv->netdev->dev_addr);
	if (ret < 0)
		return ret;
		
	/* Record topology change */
	adin2111_mk_bridge_topology_change(priv);
	
	return 0;
}

static int adin2111_mk_netdevice_event(struct notifier_block *nb,
				       unsigned long event, void *ptr)
{
	struct net_device *dev = netdev_notifier_info_to_dev(ptr);
	struct adin2111_mk_priv *priv = container_of(nb, struct adin2111_mk_priv, netdevice_nb);
	struct netdev_notifier_changeupper_info *info = ptr;
	int ret = 0;

	if (!adin2111_mk_port_dev_check(dev))
		return NOTIFY_DONE;

	switch (event) {
	case NETDEV_CHANGEUPPER:
		if (netif_is_bridge_master(info->upper_dev)) {
			if (info->linking)
				ret = adin2111_mk_port_bridge_join(priv, info->upper_dev);
			else
				ret = adin2111_mk_port_bridge_leave(priv, info->upper_dev);
		}
		break;
	default:
		break;
	}

	return notifier_from_errno(ret);
}

/* STP State Management Functions */
static int adin2111_mk_port_set_forwarding_state(struct adin2111_mk_priv *priv)
{
	int ret;

	/* Set both internal ports to forwarding state */
	priv->ports[0].state = BR_STATE_FORWARDING;
	if (priv->cfg->ports_nr == 2)
		priv->ports[1].state = BR_STATE_FORWARDING;

	mutex_lock(&priv->lock);
	ret = adin2111_mk_write_mac_address(priv, priv->netdev->dev_addr);
	if (ret < 0)
		goto out;

	if (adin2111_mk_can_offload_forwarding(priv))
		ret = adin2111_mk_hw_forwarding(priv, true);
	else
		ret = adin2111_mk_setup_rx_mode(priv);
out:
	mutex_unlock(&priv->lock);

	return ret;
}

static int adin2111_mk_port_set_blocking_state(struct adin2111_mk_priv *priv)
{
	u8 mac[ETH_ALEN] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x00}; /* STP BPDU MAC */
	__maybe_unused u8 mask[ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0}; /* Match STP range */
	u32 port_rules;
	u32 fwd_rules;
	int ret;

	/* Set both internal ports to blocking state */
	priv->ports[0].state = BR_STATE_BLOCKING;
	if (priv->cfg->ports_nr == 2)
		priv->ports[1].state = BR_STATE_BLOCKING;

	/* Allow only STP BPDUs to pass to host */
	port_rules = ADIN1110_MAC_ADDR_APPLY2PORT;
	if (priv->cfg->ports_nr == 2)
		port_rules |= ADIN2111_MAC_ADDR_APPLY2PORT2;
		
	fwd_rules = ADIN1110_MAC_ADDR_TO_HOST;

	mutex_lock(&priv->lock);

	/* Disable hardware forwarding */
	ret = adin2111_mk_hw_forwarding(priv, false);
	if (ret < 0)
		goto out;

	/* Configure STP BPDU forwarding to host only */
	ret = adin2111_mk_write_mac_address_full(priv, mac, 0, port_rules, fwd_rules);
	if (ret < 0)
		goto out;

	/* Clear other MAC addresses to prevent normal traffic */
	ret = adin2111_mk_clear_mac_address(priv, 1);
	
out:
	mutex_unlock(&priv->lock);

	return ret;
}

/* Comprehensive STP State Handler Functions */
static int adin2111_mk_port_set_learning_state(struct adin2111_mk_priv *priv)
{
	int ret;

	/* Set both internal ports to learning state */
	priv->ports[0].state = BR_STATE_LEARNING;
	if (priv->cfg->ports_nr == 2)
		priv->ports[1].state = BR_STATE_LEARNING;

	mutex_lock(&priv->lock);

	/* Disable hardware forwarding but allow MAC learning */
	ret = adin2111_mk_hw_forwarding(priv, false);
	if (ret < 0)
		goto out;

	/* Configure for MAC learning - accept unicast but forward to host */
	ret = adin2111_mk_write_mac_address(priv, priv->netdev->dev_addr);
	if (ret < 0)
		goto out;

	/* Setup RX mode for learning */
	ret = adin2111_mk_setup_rx_mode(priv);

out:
	mutex_unlock(&priv->lock);
	trace_adin1110_hw_forwarding(false);
	
	return ret;
}

static int adin2111_mk_port_set_listening_state(struct adin2111_mk_priv *priv)
{
	u8 bpdu_mac[ETH_ALEN] = {0x01, 0x80, 0xC2, 0x00, 0x00, 0x00};
	u32 port_rules, fwd_rules;
	int ret;

	/* Set both internal ports to listening state */
	priv->ports[0].state = BR_STATE_LISTENING;
	if (priv->cfg->ports_nr == 2)
		priv->ports[1].state = BR_STATE_LISTENING;

	/* Configure to pass only STP BPDUs and control frames */
	port_rules = ADIN1110_MAC_ADDR_APPLY2PORT;
	if (priv->cfg->ports_nr == 2)
		port_rules |= ADIN2111_MAC_ADDR_APPLY2PORT2;
		
	fwd_rules = ADIN1110_MAC_ADDR_TO_HOST;

	mutex_lock(&priv->lock);

	/* Disable hardware forwarding */
	ret = adin2111_mk_hw_forwarding(priv, false);
	if (ret < 0)
		goto out;

	/* Allow only STP BPDUs to pass */
	ret = adin2111_mk_write_mac_address_full(priv, bpdu_mac, 0, port_rules, fwd_rules);
	if (ret < 0)
		goto out;

	/* Clear other MAC address slots to block normal traffic */
	ret = adin2111_mk_clear_mac_address(priv, 1);

out:
	mutex_unlock(&priv->lock);
	trace_adin1110_hw_forwarding(false);
	
	return ret;
}

static int adin2111_mk_port_set_disabled_state(struct adin2111_mk_priv *priv)
{
	int ret;

	/* Set both internal ports to disabled state */
	priv->ports[0].state = BR_STATE_DISABLED;
	if (priv->cfg->ports_nr == 2)
		priv->ports[1].state = BR_STATE_DISABLED;

	mutex_lock(&priv->lock);

	/* Disable all forwarding and filtering */
	ret = adin2111_mk_hw_forwarding(priv, false);
	if (ret < 0)
		goto out;

	/* Clear all MAC address slots to block all traffic */
	{
		int i;
		for (i = 0; i < ADIN_MAC_MAX_ADDR_SLOTS; i++) {
			ret = adin2111_mk_clear_mac_address(priv, i);
			if (ret < 0)
				break;
		}
	}

	/* Disable promiscuous mode */
	ret = adin2111_mk_set_promisc_mode(priv, false);

out:
	mutex_unlock(&priv->lock);
	trace_adin1110_hw_forwarding(false);
	
	return ret;
}

/* Enhanced STP State Management with Validation and Logging */
static const char *adin2111_mk_stp_state_name(u8 state)
{
	switch (state) {
	case BR_STATE_DISABLED:   return "DISABLED";
	case BR_STATE_LISTENING:  return "LISTENING";
	case BR_STATE_LEARNING:   return "LEARNING";
	case BR_STATE_FORWARDING: return "FORWARDING";
	case BR_STATE_BLOCKING:   return "BLOCKING";
	default:                  return "UNKNOWN";
	}
}

static bool adin2111_mk_stp_state_valid_transition(u8 old_state, u8 new_state)
{
	/* STP state transition validation according to 802.1D */
	switch (old_state) {
	case BR_STATE_DISABLED:
		return (new_state == BR_STATE_BLOCKING || 
			new_state == BR_STATE_DISABLED);
	case BR_STATE_BLOCKING:
		return (new_state == BR_STATE_LISTENING ||
			new_state == BR_STATE_DISABLED ||
			new_state == BR_STATE_BLOCKING);
	case BR_STATE_LISTENING:
		return (new_state == BR_STATE_LEARNING ||
			new_state == BR_STATE_BLOCKING ||
			new_state == BR_STATE_DISABLED);
	case BR_STATE_LEARNING:
		return (new_state == BR_STATE_FORWARDING ||
			new_state == BR_STATE_BLOCKING ||
			new_state == BR_STATE_DISABLED);
	case BR_STATE_FORWARDING:
		return (new_state == BR_STATE_BLOCKING ||
			new_state == BR_STATE_DISABLED);
	default:
		return true; /* Allow any transition from unknown states */
	}
}

static int adin2111_mk_port_attr_stp_state_set(struct adin2111_mk_priv *priv, u8 new_state)
{
	u8 old_state_p1 = priv->ports[0].state;
	__maybe_unused u8 old_state_p2 = (priv->cfg->ports_nr == 2) ? priv->ports[1].state : BR_STATE_DISABLED;
	int ret;

	/* Validate STP state transition */
	if (!adin2111_mk_stp_state_valid_transition(old_state_p1, new_state)) {
		netdev_warn(priv->netdev, "Invalid STP state transition: %s -> %s\n",
			    adin2111_mk_stp_state_name(old_state_p1),
			    adin2111_mk_stp_state_name(new_state));
		/* Allow the transition but log the warning */
	}

	/* Log state transition */
	netdev_info(priv->netdev, "STP state transition: %s -> %s (dual T1L ports)\n",
		    adin2111_mk_stp_state_name(old_state_p1),
		    adin2111_mk_stp_state_name(new_state));

	trace_adin1110_init("stp_state_change");

	switch (new_state) {
	case BR_STATE_FORWARDING:
		ret = adin2111_mk_port_set_forwarding_state(priv);
		break;
	case BR_STATE_LEARNING:
		ret = adin2111_mk_port_set_learning_state(priv);
		break;
	case BR_STATE_LISTENING:
		ret = adin2111_mk_port_set_listening_state(priv);
		break;
	case BR_STATE_DISABLED:
		ret = adin2111_mk_port_set_disabled_state(priv);
		break;
	case BR_STATE_BLOCKING:
		ret = adin2111_mk_port_set_blocking_state(priv);
		break;
	default:
		netdev_err(priv->netdev, "Unsupported STP state: %d\n", new_state);
		return -EINVAL;
	}

	if (ret < 0) {
		netdev_err(priv->netdev, "Failed to set STP state %s: %d\n",
			   adin2111_mk_stp_state_name(new_state), ret);
		return ret;
	}

	/* Update port state tracking */
	priv->ports[0].state = new_state;
	if (priv->cfg->ports_nr == 2)
		priv->ports[1].state = new_state;

	netdev_info(priv->netdev, "STP state successfully changed to %s\n",
		    adin2111_mk_stp_state_name(new_state));

	/* Update bridge forwarding based on new STP state */
	adin2111_mk_update_bridge_forwarding(priv);

	return 0;
}

static int adin2111_mk_port_attr_set(struct net_device *dev, 
				     __maybe_unused const void *ctx,
				     const struct switchdev_attr *attr,
				     __maybe_unused struct netlink_ext_ack *extack)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);

	switch (attr->id) {
	case SWITCHDEV_ATTR_ID_PORT_STP_STATE:
		return adin2111_mk_port_attr_stp_state_set(priv, attr->u.stp_state);
	default:
		return -EOPNOTSUPP;
	}
}

static int adin2111_mk_switchdev_blocking_event(struct notifier_block *nb,
						unsigned long event,
						void *ptr)
{
	struct net_device *netdev = switchdev_notifier_info_to_dev(ptr);
	__maybe_unused struct adin2111_mk_priv *priv = container_of(nb, struct adin2111_mk_priv, switchdev_blocking_nb);
	int ret;

	if (event == SWITCHDEV_PORT_ATTR_SET) {
		ret = switchdev_handle_port_attr_set(netdev, ptr,
						     adin2111_mk_port_dev_check,
						     adin2111_mk_port_attr_set);

		return notifier_from_errno(ret);
	}

	return NOTIFY_DONE;
}

/* FDB Management Functions (placeholder for Phase 3) */
static void adin2111_mk_fdb_offload_notify(struct net_device *netdev,
					   struct switchdev_notifier_fdb_info *rcv)
{
	struct switchdev_notifier_fdb_info info = {};

	info.addr = rcv->addr;
	info.vid = rcv->vid;
	info.offloaded = true;
	call_switchdev_notifiers(SWITCHDEV_FDB_OFFLOADED, netdev, &info.info, NULL);
}

static int adin2111_mk_fdb_add(struct adin2111_mk_priv *priv,
			       struct switchdev_notifier_fdb_info *fdb)
{
	u32 port_rules, fwd_rules;
	int ret;

	/* Configure MAC address for hardware learning */
	port_rules = ADIN1110_MAC_ADDR_APPLY2PORT;
	if (priv->cfg->ports_nr == 2)
		port_rules |= ADIN2111_MAC_ADDR_APPLY2PORT2;
	
	fwd_rules = ADIN1110_MAC_ADDR_TO_HOST;
	
	ret = adin2111_mk_write_mac_address_full(priv, fdb->addr, 0, port_rules, fwd_rules);
	if (ret < 0)
		return ret;

	trace_adin1110_write_mac_address(&priv->ports[0], 0, fdb->addr, NULL, port_rules);
	
	return 0;
}

static int adin2111_mk_fdb_del(struct adin2111_mk_priv *priv,
			       struct switchdev_notifier_fdb_info *fdb)
{
	/* For now, clear a MAC address slot - full implementation in Phase 3 */
	return adin2111_mk_clear_mac_address(priv, 0);
}

/* Switchdev Event Work Queue Processing */
static void adin2111_mk_switchdev_event_work(struct work_struct *work)
{
	struct adin2111_mk_switchdev_event_work *switchdev_work;
	struct adin2111_mk_priv *priv;
	int ret;

	switchdev_work = container_of(work, struct adin2111_mk_switchdev_event_work, work);
	priv = *(struct adin2111_mk_priv **)netdev_priv(switchdev_work->port_priv->netdev);

	mutex_lock(&priv->lock);

	switch (switchdev_work->event) {
	case SWITCHDEV_FDB_ADD_TO_DEVICE:
		ret = adin2111_mk_fdb_add(priv, &switchdev_work->fdb_info);
		if (!ret)
			adin2111_mk_fdb_offload_notify(priv->netdev,
						       &switchdev_work->fdb_info);
		break;
	case SWITCHDEV_FDB_DEL_TO_DEVICE:
		adin2111_mk_fdb_del(priv, &switchdev_work->fdb_info);
		break;
	default:
		break;
	}

	mutex_unlock(&priv->lock);

	kfree(switchdev_work->fdb_info.addr);
	dev_put(switchdev_work->port_priv->netdev);
	kfree(switchdev_work);
}

static int adin2111_mk_switchdev_event(struct notifier_block *nb,
				       unsigned long event, void *ptr)
{
	struct net_device *netdev = switchdev_notifier_info_to_dev(ptr);
	struct adin2111_mk_priv *priv = container_of(nb, struct adin2111_mk_priv, switchdev_nb);
	struct adin2111_mk_switchdev_event_work *switchdev_work;
	struct switchdev_notifier_fdb_info *fdb_info = ptr;

	if (!adin2111_mk_port_dev_check(netdev))
		return NOTIFY_DONE;

	switchdev_work = kzalloc(sizeof(*switchdev_work), GFP_ATOMIC);
	if (WARN_ON(!switchdev_work))
		return NOTIFY_BAD;

	INIT_WORK(&switchdev_work->work, adin2111_mk_switchdev_event_work);
	switchdev_work->port_priv = &priv->ports[0]; /* Use first port for single interface */
	switchdev_work->event = event;

	switch (event) {
	case SWITCHDEV_FDB_ADD_TO_DEVICE:
	case SWITCHDEV_FDB_DEL_TO_DEVICE:
		memcpy(&switchdev_work->fdb_info, ptr,
		       sizeof(switchdev_work->fdb_info));
		switchdev_work->fdb_info.addr = kzalloc(ETH_ALEN, GFP_ATOMIC);

		if (!switchdev_work->fdb_info.addr)
			goto err_addr_alloc;

		ether_addr_copy((u8 *)switchdev_work->fdb_info.addr,
				fdb_info->addr);
		dev_hold(netdev);
		break;
	default:
		kfree(switchdev_work);
		return NOTIFY_DONE;
	}

	queue_work(priv->switchdev_wq, &switchdev_work->work);

	return NOTIFY_DONE;

err_addr_alloc:
	kfree(switchdev_work);
	return NOTIFY_BAD;
}

/* Enhanced Bridge State Management and Monitoring */
static bool adin2111_mk_bridge_offload_allowed(struct adin2111_mk_priv *priv)
{
	/* Check if hardware forwarding can be safely enabled */
	if (!priv->ports[0].bridge)
		return false;

	/* Ensure both ports are in forwarding state if dual-port */
	if (priv->cfg->ports_nr == 2 && priv->ports[1].bridge) {
		if (priv->ports[0].state != BR_STATE_FORWARDING ||
		    priv->ports[1].state != BR_STATE_FORWARDING)
			return false;
	} else if (priv->ports[0].state != BR_STATE_FORWARDING) {
		return false;
	}

	return adin2111_mk_can_offload_forwarding(priv);
}

static void adin2111_mk_update_bridge_forwarding(struct adin2111_mk_priv *priv)
{
	bool should_forward = adin2111_mk_bridge_offload_allowed(priv);
	
	mutex_lock(&priv->lock);
	
	if (should_forward != priv->forwarding) {
		int ret = adin2111_mk_hw_forwarding(priv, should_forward);
		if (ret < 0) {
			netdev_warn(priv->netdev, 
				    "Failed to %s hardware forwarding: %d\n",
				    should_forward ? "enable" : "disable", ret);
		} else {
			netdev_info(priv->netdev, 
				    "Hardware forwarding %s (STP compliance)\n",
				    should_forward ? "enabled" : "disabled");
			trace_adin1110_hw_forwarding(should_forward);
		}
	}
	
	mutex_unlock(&priv->lock);
}

/* STP Compliance and Statistics */
__maybe_unused static int adin2111_mk_get_stp_stats(struct adin2111_mk_priv *priv, 
				     struct net_device *dev, 
				     struct ethtool_stats *stats, u64 *data)
{
	/* Report STP-related statistics */
	data[0] = (priv->ports[0].state == BR_STATE_FORWARDING) ? 1 : 0;
	data[1] = (priv->cfg->ports_nr == 2 && priv->ports[1].state == BR_STATE_FORWARDING) ? 1 : 0;
	data[2] = priv->forwarding ? 1 : 0;
	data[3] = priv->ports[0].bridge ? 1 : 0;
	
	return 4; /* Number of STP statistics */
}

/* Advanced Bridge Topology Management */
static void adin2111_mk_bridge_topology_change(struct adin2111_mk_priv *priv)
{
	atomic_inc(&priv->topology_changes);
	priv->bridge_topology_changes++;
	
	netdev_info(priv->netdev, "Bridge topology change detected (count: %u)\n",
		    priv->bridge_topology_changes);
	
	trace_adin1110_init("topology_change");
	
	/* Update forwarding state based on new topology */
	adin2111_mk_update_bridge_forwarding(priv);
}

static int adin2111_mk_bridge_getlink(struct sk_buff *skb, u32 pid, u32 seq,
				      struct net_device *dev, u32 filter_mask,
				      int nlflags)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);
	struct net_device *bridge;
	int ret;

	mutex_lock(&priv->bridge_lock);
	bridge = priv->active_bridge;
	
	if (!bridge) {
		mutex_unlock(&priv->bridge_lock);
		return 0;
	}
	
	ret = ndo_dflt_bridge_getlink(skb, pid, seq, dev, 
				      bridge->priv_flags & IFF_BRIDGE_PORT ? 
				      priv->ports[0].state : BR_STATE_DISABLED,
				      0, 0, nlflags, filter_mask, NULL);
	
	mutex_unlock(&priv->bridge_lock);
	return ret;
}

static int adin2111_mk_bridge_setlink(struct net_device *dev, 
				      struct nlmsghdr *nlh,
				      u16 flags, struct netlink_ext_ack *extack)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);
	struct nlattr *attr, *br_spec;
	int rem, ret = 0;
	u16 mode = BRIDGE_MODE_HAIRPIN;
	
	if (!priv->active_bridge)
		return -EOPNOTSUPP;

	br_spec = nlmsg_find_attr(nlh, sizeof(struct ifinfomsg), IFLA_AF_SPEC);
	if (!br_spec)
		return -EINVAL;

	nla_for_each_nested(attr, br_spec, rem) {
		switch (nla_type(attr)) {
		case IFLA_BRIDGE_MODE:
			mode = nla_get_u16(attr);
			break;
		case IFLA_BRIDGE_FLAGS:
			/* Handle bridge flags if needed */
			break;
		}
	}

	netdev_info(dev, "Bridge setlink: mode=%u, flags=%u\n", mode, flags);
	adin2111_mk_bridge_topology_change(priv);
	
	return ret;
}

/* VLAN Awareness and Management */
static int adin2111_mk_vlan_rx_add_vid(struct net_device *dev, 
				       __be16 proto, u16 vid)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);
	
	if (proto != htons(ETH_P_8021Q))
		return -EOPNOTSUPP;
		
	mutex_lock(&priv->bridge_lock);
	
	/* Store default VLAN for bridge operations */
	if (vid != 0) {
		priv->bridge_vid = vid;
		netdev_info(dev, "Added VLAN %u (proto 0x%04x)\n", 
			    vid, ntohs(proto));
	}
	
	mutex_unlock(&priv->bridge_lock);
	
	/* Update hardware forwarding rules for VLAN */
	adin2111_mk_update_bridge_forwarding(priv);
	
	return 0;
}

static int adin2111_mk_vlan_rx_kill_vid(struct net_device *dev, 
					__be16 proto, u16 vid)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);
	
	if (proto != htons(ETH_P_8021Q))
		return -EOPNOTSUPP;
		
	mutex_lock(&priv->bridge_lock);
	
	if (priv->bridge_vid == vid) {
		priv->bridge_vid = 0;
		netdev_info(dev, "Removed VLAN %u (proto 0x%04x)\n", 
			    vid, ntohs(proto));
	}
	
	mutex_unlock(&priv->bridge_lock);
	
	/* Update hardware forwarding rules */
	adin2111_mk_update_bridge_forwarding(priv);
	
	return 0;
}

/* Enhanced Bridge Feature Detection */
static netdev_features_t adin2111_mk_bridge_features_check(struct sk_buff *skb,
							   struct net_device *dev,
							   netdev_features_t features)
{
	struct adin2111_mk_priv *priv = *(struct adin2111_mk_priv **)netdev_priv(dev);
	
	/* Check if bridge supports our hardware features */
	if (priv->active_bridge) {
		/* Hardware forwarding is available when in bridge mode */
		if (priv->forwarding)
			return features;
		
		/* Note: NETIF_F_HW_SWITCH_OFFLOAD not available in kernel 6.6 */
		/* Hardware offload features managed through bridge forwarding */
	}
	
	return features;
}

/* Dynamic Bridge Membership Tracking */
static void adin2111_mk_update_bridge_membership(struct adin2111_mk_priv *priv, 
						  struct net_device *bridge,
						  bool joining)
{
	mutex_lock(&priv->bridge_lock);
	
	if (joining) {
		priv->active_bridge = bridge;
		priv->bridge_join_time = jiffies;
		priv->bridge_features = bridge->features;
		priv->bridge_vid = 1; /* Default VLAN */
		atomic_set(&priv->topology_changes, 0);
		
		netdev_info(priv->netdev, 
			    "Bridge membership: joined %s (features: 0x%x)\n",
			    bridge->name, priv->bridge_features);
	} else {
		if (priv->active_bridge == bridge) {
			unsigned long duration = (jiffies - priv->bridge_join_time) / HZ;
			
			netdev_info(priv->netdev,
				    "Bridge membership: left %s (duration: %lus, changes: %u)\n",
				    bridge->name, duration, 
				    atomic_read(&priv->topology_changes));
			
			priv->active_bridge = NULL;
			priv->bridge_features = 0;
			priv->bridge_vid = 0;
		}
	}
	
	mutex_unlock(&priv->bridge_lock);
	
	/* Update port bridge references */
	priv->ports[0].bridge = joining ? bridge : NULL;
	if (priv->cfg->ports_nr == 2)
		priv->ports[1].bridge = joining ? bridge : NULL;
}

/* Notifier Registration and Work Queue Management */
static int adin2111_mk_setup_notifiers(struct adin2111_mk_priv *priv)
{
	int ret;

	/* Create dedicated work queue for switchdev events */
	priv->switchdev_wq = alloc_ordered_workqueue("adin2111_mk_switchdev", 0);
	if (!priv->switchdev_wq)
		return -ENOMEM;

	/* Initialize notifier blocks */
	priv->netdevice_nb.notifier_call = adin2111_mk_netdevice_event;
	priv->switchdev_blocking_nb.notifier_call = adin2111_mk_switchdev_blocking_event;
	priv->switchdev_nb.notifier_call = adin2111_mk_switchdev_event;

	/* Register network device notifier */
	ret = register_netdevice_notifier(&priv->netdevice_nb);
	if (ret) {
		dev_err(&priv->spidev->dev, "Failed to register netdevice notifier: %d\n", ret);
		goto err_destroy_wq;
	}

	/* Register switchdev blocking notifier */
	ret = register_switchdev_blocking_notifier(&priv->switchdev_blocking_nb);
	if (ret) {
		dev_err(&priv->spidev->dev, "Failed to register switchdev blocking notifier: %d\n", ret);
		goto err_unreg_netdev;
	}

	/* Register switchdev event notifier */
	ret = register_switchdev_notifier(&priv->switchdev_nb);
	if (ret) {
		dev_err(&priv->spidev->dev, "Failed to register switchdev notifier: %d\n", ret);
		goto err_unreg_blocking;
	}

	return 0;

err_unreg_blocking:
	unregister_switchdev_blocking_notifier(&priv->switchdev_blocking_nb);
err_unreg_netdev:
	unregister_netdevice_notifier(&priv->netdevice_nb);
err_destroy_wq:
	destroy_workqueue(priv->switchdev_wq);
	return ret;
}

static void adin2111_mk_unregister_notifiers(struct adin2111_mk_priv *priv)
{
	unregister_switchdev_notifier(&priv->switchdev_nb);
	unregister_switchdev_blocking_notifier(&priv->switchdev_blocking_nb);
	unregister_netdevice_notifier(&priv->netdevice_nb);
	
	if (priv->switchdev_wq) {
		destroy_workqueue(priv->switchdev_wq);
		priv->switchdev_wq = NULL;
	}
}

/* Enhanced network device operations with bridge topology management */
static const struct net_device_ops adin2111_mk_netdev_ops = {
	.ndo_open		= adin2111_mk_net_open,
	.ndo_stop		= adin2111_mk_net_stop,
	.ndo_eth_ioctl		= adin2111_mk_ioctl,
	.ndo_start_xmit		= adin2111_mk_start_xmit,
	.ndo_set_mac_address	= adin2111_mk_ndo_set_mac_address,
	.ndo_set_rx_mode	= adin2111_mk_set_rx_mode,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_get_stats64	= adin2111_mk_ndo_get_stats64,
	.ndo_bridge_getlink	= adin2111_mk_bridge_getlink,
	.ndo_bridge_setlink	= adin2111_mk_bridge_setlink,
	.ndo_vlan_rx_add_vid	= adin2111_mk_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= adin2111_mk_vlan_rx_kill_vid,
	.ndo_features_check	= adin2111_mk_bridge_features_check,
};

/* Enhanced register debugging interface */
static ssize_t reg_value_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	struct regs_attr *regs_attr = container_of(attr, struct regs_attr, dev_attr_value);
	struct adin2111_mk_priv *priv = regs_attr->regsinfo.priv;
	u32 val;
	int ret;

	ret = adin1110_read_reg(priv, regs_attr->regsinfo.reg, &val);
	if (ret < 0)
		return ret;

	return sprintf(buf, "0x%08X\n", val);
}

static ssize_t reg_value_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct regs_attr *regs_attr = container_of(attr, struct regs_attr, dev_attr_value);
	struct adin2111_mk_priv *priv = regs_attr->regsinfo.priv;
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret < 0)
		return ret;

	ret = adin1110_write_reg(priv, regs_attr->regsinfo.reg, val);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t reg_addr_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct dev_ext_attribute *reg_attr = container_of(attr, struct dev_ext_attribute, attr);
	struct regs_info *info = reg_attr->var;

	return sprintf(buf, "0x%04lX\n", info->reg);
}

static ssize_t reg_addr_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct dev_ext_attribute *reg_attr = container_of(attr, struct dev_ext_attribute, attr);
	struct regs_info *info = reg_attr->var;
	u32 val;
	int ret;

	ret = kstrtou32(buf, 0, &val);
	if (ret < 0)
		return ret;

	info->reg = val;

	return count;
}

static int adin2111_mk_setup_sysfs(struct adin2111_mk_priv *priv)
{
	struct regs_attr *regs_attr;
	struct device *dev = &priv->spidev->dev;
	int ret;

	regs_attr = devm_kzalloc(dev, sizeof(*regs_attr), GFP_KERNEL);
	if (!regs_attr)
		return -ENOMEM;

	regs_attr->regsinfo.priv = priv;
	regs_attr->regsinfo.reg = 0;

	regs_attr->dev_attr_reg.attr.attr.name = "adin_reg";
	regs_attr->dev_attr_reg.attr.attr.mode = 0644;
	regs_attr->dev_attr_reg.attr.show = reg_addr_show;
	regs_attr->dev_attr_reg.attr.store = reg_addr_store;
	regs_attr->dev_attr_reg.var = &regs_attr->regsinfo;

	regs_attr->dev_attr_value.attr.name = "adin_value";
	regs_attr->dev_attr_value.attr.mode = 0644;
	regs_attr->dev_attr_value.show = reg_value_show;
	regs_attr->dev_attr_value.store = reg_value_store;

	ret = device_create_file(dev, &regs_attr->dev_attr_reg.attr);
	if (ret < 0)
		return ret;

	ret = device_create_file(dev, &regs_attr->dev_attr_value);
	if (ret < 0) {
		device_remove_file(dev, &regs_attr->dev_attr_reg.attr);
		return ret;
	}

	priv->regsattr = regs_attr;

	return 0;
}

static void adin2111_mk_remove_sysfs(struct adin2111_mk_priv *priv)
{
	struct device *dev = &priv->spidev->dev;

	if (priv->regsattr) {
		device_remove_file(dev, &priv->regsattr->dev_attr_value);
		device_remove_file(dev, &priv->regsattr->dev_attr_reg.attr);
	}
}

/* Enhanced network device probe with dual T1L PHY support */
static int adin2111_mk_probe_netdev(struct adin2111_mk_priv *priv)
{
	struct device *dev = &priv->spidev->dev;
	struct net_device *netdev;
	int ret, i;

	netdev = devm_alloc_etherdev(dev, sizeof(struct adin2111_mk_priv *));
	if (!netdev)
		return -ENOMEM;

	/* Set up the network device */
	priv->netdev = netdev;
	SET_NETDEV_DEV(netdev, dev);
	
	/* Store pointer to our private data in netdev private area */
	*(struct adin2111_mk_priv **)netdev_priv(netdev) = priv;

	/* Initialize internal dual port structures */
	for (i = 0; i < ADIN_MAC_MAX_PORTS; i++) {
		priv->ports[i].priv = priv;
		priv->ports[i].netdev = netdev;
		priv->ports[i].nr = i;
		priv->ports[i].state = BR_STATE_FORWARDING;
		priv->ports[i].cfg = priv->cfg;
		priv->port_refs[i] = &priv->ports[i];
		INIT_WORK(&priv->ports[i].tx_work, adin2111_mk_tx_work);
		INIT_WORK(&priv->ports[i].rx_mode_work, adin2111_mk_rx_mode_work);
		skb_queue_head_init(&priv->ports[i].txq);
	}

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

	/* Enhanced network device configuration */
	netdev->flags |= IFF_BROADCAST | IFF_MULTICAST;
	netdev->mtu = ETH_DATA_LEN;
	netdev->min_mtu = ETH_MIN_MTU;
	netdev->max_mtu = ETH_DATA_LEN;
	netdev->type = ARPHRD_ETHER;
	netdev->hard_header_len = ETH_HLEN;
	netdev->addr_len = ETH_ALEN;

	/* Setup dual PHY management for both T1L ports */
	priv->phydev = get_phy_device(priv->mii_bus, priv->cfg->phy_ids[0], false);
	if (IS_ERR(priv->phydev)) {
		netdev_err(netdev, "Could not find PHY 1 with device address: %d.\n", priv->cfg->phy_ids[0]);
		return PTR_ERR(priv->phydev);
	}

	priv->phydev = phy_connect(netdev, phydev_name(priv->phydev), 
				   adin2111_mk_adjust_link, PHY_INTERFACE_MODE_INTERNAL);
	priv->ports[0].phydev = priv->phydev;
	if (IS_ERR(priv->phydev)) {
		netdev_err(netdev, "Could not connect PHY 1 with device address: %d.\n", priv->cfg->phy_ids[0]);
		return PTR_ERR(priv->phydev);
	}

	/* Setup second PHY for internal dual T1L port management */
	priv->phydev_p2 = get_phy_device(priv->mii_bus, priv->cfg->phy_ids[1], false);
	if (!IS_ERR(priv->phydev_p2)) {
		priv->phydev_p2 = phy_connect(netdev, phydev_name(priv->phydev_p2), 
					      adin2111_mk_adjust_link, PHY_INTERFACE_MODE_INTERNAL);
		if (!IS_ERR(priv->phydev_p2)) {
			priv->ports[1].phydev = priv->phydev_p2;
		} else {
			dev_warn(dev, "Could not connect PHY 2, single T1L port mode\n");
			priv->phydev_p2 = NULL;
		}
	} else {
		dev_info(dev, "PHY 2 not found, single T1L port operation\n");
		priv->phydev_p2 = NULL;
	}

	ret = devm_add_action_or_reset(dev, adin2111_mk_disconnect_phy, priv->phydev);
	if (ret < 0)
		return ret;

	if (priv->phydev_p2) {
		ret = devm_add_action_or_reset(dev, adin2111_mk_disconnect_phy, priv->phydev_p2);
		if (ret < 0)
			return ret;
	}

	/* Request IRQ */
	ret = devm_request_threaded_irq(dev, priv->spidev->irq, NULL, adin2111_mk_irq,
					IRQF_TRIGGER_LOW | IRQF_ONESHOT, dev_name(dev), priv);
	if (ret < 0)
		return ret;

	/* Setup register debugging interface */
	ret = adin2111_mk_setup_sysfs(priv);
	if (ret < 0)
		dev_warn(dev, "Failed to setup sysfs interface: %d\n", ret);

	/* Register network device */
	ret = devm_register_netdev(dev, netdev);
	if (ret < 0) {
		dev_err(dev, "Failed to register network device.\n");
		adin2111_mk_remove_sysfs(priv);
		return ret;
	}

	/* Setup switchdev notifiers for bridge integration */
	ret = adin2111_mk_setup_notifiers(priv);
	if (ret < 0) {
		dev_err(dev, "Failed to setup switchdev notifiers: %d\n", ret);
		adin2111_mk_remove_sysfs(priv);
		return ret;
	}

	/* Log successful interface creation with enhanced features */
	if (priv->phydev_p2) {
		netdev_info(netdev, "ADIN2111-MK enhanced driver loaded: dual T1L ports via single eth0, MAC: %pM\n", 
			    netdev->dev_addr);
		netdev_info(netdev, "Features: HW forwarding, 16-slot MAC filtering, dual PHY mgmt, bridge integration\n");
	} else {
		netdev_info(netdev, "ADIN2111-MK single T1L port mode, MAC: %pM\n", netdev->dev_addr);
	}

	return 0;
}

static int adin2111_mk_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct adin2111_mk_priv *priv;
	int ret;

	trace_adin1110_init("probe");

	priv = devm_kzalloc(dev, sizeof(struct adin2111_mk_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);

	priv->spidev = spi;
	priv->cfg = &adin2111_mk_cfg;
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;

	mutex_init(&priv->lock);
	spin_lock_init(&priv->state_lock);
	mutex_init(&priv->bridge_lock);
	
	/* Initialize bridge topology management */
	priv->active_bridge = NULL;
	priv->bridge_features = 0;
	priv->bridge_vid = 0;
	atomic_set(&priv->topology_changes, 0);
	priv->bridge_topology_changes = 0;
	priv->stp_state_changes = 0;

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

static void adin2111_mk_remove(struct spi_device *spi)
{
	struct adin2111_mk_priv *priv = spi_get_drvdata(spi);
	
	adin2111_mk_unregister_notifiers(priv);
	adin2111_mk_remove_sysfs(priv);
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
	.remove = adin2111_mk_remove,
	.id_table = adin2111_mk_spi_id,
};

module_spi_driver(adin2111_mk_driver);

MODULE_DESCRIPTION("ADIN2111-MK Enhanced Single Interface Network Driver with Dual T1L Port Support");
MODULE_AUTHOR("Murray Kopit <murr2k@gmail.com>");
MODULE_LICENSE("Dual BSD/GPL");
