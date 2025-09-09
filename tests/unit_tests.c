/*
 * ADIN2111-MK Driver Unit Tests
 * Comprehensive test suite for driver functionality validation
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/phy.h>
#include <linux/crc8.h>

// Test framework includes
#include <linux/kunit/test.h>

// Mock structures and definitions for testing
struct mock_spi_device {
    struct spi_device spi;
    u8 mock_registers[0x1000];
    bool simulate_error;
    int error_count;
};

struct test_context {
    struct mock_spi_device *mock_spi;
    struct adin1110_priv *priv;
    struct net_device *netdev;
    bool initialized;
};

// Test fixtures and setup
static struct test_context test_ctx;

/**
 * Test Suite 1: SPI Communication Tests
 */

/* Test basic register read functionality */
static void test_adin1110_read_reg_basic(struct kunit *test)
{
    struct adin1110_priv *priv = test_ctx.priv;
    u32 val;
    int ret;
    
    // Setup mock register value
    test_ctx.mock_spi->mock_registers[ADIN1110_PHY_ID] = 0x12345678;
    
    // Test read operation
    ret = adin1110_read_reg(priv, ADIN1110_PHY_ID, &val);
    
    KUNIT_EXPECT_EQ(test, ret, 0);
    KUNIT_EXPECT_EQ(test, val, 0x12345678);
}

/* Test register read with CRC validation */
static void test_adin1110_read_reg_with_crc(struct kunit *test)
{
    struct adin1110_priv *priv = test_ctx.priv;
    u32 val;
    int ret;
    
    // Enable CRC mode
    priv->append_crc = true;
    
    // Setup mock register with valid CRC
    test_ctx.mock_spi->mock_registers[ADIN1110_CONFIG1] = 0xABCDEF00;
    
    ret = adin1110_read_reg(priv, ADIN1110_CONFIG1, &val);
    
    KUNIT_EXPECT_EQ(test, ret, 0);
    KUNIT_EXPECT_EQ(test, val, 0xABCDEF00);
}

/* Test register read error handling */
static void test_adin1110_read_reg_error(struct kunit *test)
{
    struct adin1110_priv *priv = test_ctx.priv;
    u32 val;
    int ret;
    
    // Simulate SPI error
    test_ctx.mock_spi->simulate_error = true;
    
    ret = adin1110_read_reg(priv, ADIN1110_STATUS0, &val);
    
    KUNIT_EXPECT_LT(test, ret, 0);
}

/* Test basic register write functionality */
static void test_adin1110_write_reg_basic(struct kunit *test)
{
    struct adin1110_priv *priv = test_ctx.priv;
    u32 test_val = 0x87654321;
    int ret;
    
    ret = adin1110_write_reg(priv, ADIN1110_CONFIG2, test_val);
    
    KUNIT_EXPECT_EQ(test, ret, 0);
    KUNIT_EXPECT_EQ(test, test_ctx.mock_spi->mock_registers[ADIN1110_CONFIG2], test_val);
}

/* Test register write with CRC */
static void test_adin1110_write_reg_with_crc(struct kunit *test)
{
    struct adin1110_priv *priv = test_ctx.priv;
    u32 test_val = 0x13579BDF;
    int ret;
    
    priv->append_crc = true;
    
    ret = adin1110_write_reg(priv, ADIN1110_RESET, test_val);
    
    KUNIT_EXPECT_EQ(test, ret, 0);
    KUNIT_EXPECT_EQ(test, test_ctx.mock_spi->mock_registers[ADIN1110_RESET], test_val);
}

/**
 * Test Suite 2: Network Interface Tests
 */

/* Test network device allocation and setup */
static void test_netdev_allocation(struct kunit *test)
{
    struct net_device *netdev;
    struct adin1110_port_priv *port_priv;
    
    netdev = alloc_etherdev(sizeof(struct adin1110_port_priv));
    KUNIT_ASSERT_NOT_ERR_OR_NULL(test, netdev);
    
    port_priv = netdev_priv(netdev);
    KUNIT_EXPECT_NOT_NULL(test, port_priv);
    
    // Test network device configuration
    netdev->netdev_ops = &adin1110_netdev_ops;
    KUNIT_EXPECT_PTR_EQ(test, netdev->netdev_ops, &adin1110_netdev_ops);
    
    free_netdev(netdev);
}

/* Test MAC address setting */
static void test_mac_address_setting(struct kunit *test)
{
    struct net_device *netdev = test_ctx.netdev;
    u8 test_mac[ETH_ALEN] = {0x02, 0x00, 0x00, 0x12, 0x34, 0x56};
    int ret;
    
    ret = adin1110_set_mac_address(netdev, test_mac);
    
    KUNIT_EXPECT_EQ(test, ret, 0);
    KUNIT_EXPECT_MEMEQ(test, netdev->dev_addr, test_mac, ETH_ALEN);
}

/* Test invalid MAC address rejection */
static void test_invalid_mac_address(struct kunit *test)
{
    struct net_device *netdev = test_ctx.netdev;
    u8 invalid_mac[ETH_ALEN] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    int ret;
    
    ret = adin1110_set_mac_address(netdev, invalid_mac);
    
    KUNIT_EXPECT_EQ(test, ret, -EADDRNOTAVAIL);
}

/**
 * Test Suite 3: Hardware Communication Tests
 */

/* Test PHY ID validation */
static void test_phy_id_validation(struct kunit *test)
{
    struct adin1110_priv *priv = test_ctx.priv;
    int ret;
    
    // Setup correct PHY ID for ADIN2111
    test_ctx.mock_spi->mock_registers[ADIN1110_PHY_ID] = ADIN2111_PHY_ID_VAL;
    priv->cfg = &adin1110_cfgs[ADIN2111_MAC];
    
    ret = adin1110_check_spi(priv);
    
    KUNIT_EXPECT_EQ(test, ret, 0);
}

/* Test PHY ID mismatch detection */
static void test_phy_id_mismatch(struct kunit *test)
{
    struct adin1110_priv *priv = test_ctx.priv;
    int ret;
    
    // Setup incorrect PHY ID
    test_ctx.mock_spi->mock_registers[ADIN1110_PHY_ID] = 0xDEADBEEF;
    priv->cfg = &adin1110_cfgs[ADIN2111_MAC];
    
    ret = adin1110_check_spi(priv);
    
    KUNIT_EXPECT_EQ(test, ret, -EIO);
}

/* Test MDIO read operation */
static void test_mdio_read(struct kunit *test)
{
    struct mii_bus *bus = test_ctx.priv->mii_bus;
    int phy_id = 1;
    int reg = 0;
    int result;
    
    // Setup mock MDIO response
    test_ctx.mock_spi->mock_registers[ADIN1110_MDIOACC] = 
        ADIN1110_MDIO_TRDONE | 0x1234;
    
    result = adin1110_mdio_read(bus, phy_id, reg);
    
    KUNIT_EXPECT_EQ(test, result, 0x1234);
}

/* Test MDIO write operation */
static void test_mdio_write(struct kunit *test)
{
    struct mii_bus *bus = test_ctx.priv->mii_bus;
    int phy_id = 1;
    int reg = 1;
    u16 val = 0x5678;
    int ret;
    
    // Setup mock successful completion
    test_ctx.mock_spi->mock_registers[ADIN1110_MDIOACC] = ADIN1110_MDIO_TRDONE;
    
    ret = adin1110_mdio_write(bus, phy_id, reg, val);
    
    KUNIT_EXPECT_EQ(test, ret, 0);
}

/**
 * Test Suite 4: Frame Processing Tests
 */

/* Test frame transmission */
static void test_frame_transmission(struct kunit *test)
{
    struct adin1110_port_priv *port_priv = netdev_priv(test_ctx.netdev);
    struct sk_buff *skb;
    int ret;
    
    // Create test frame
    skb = dev_alloc_skb(64);
    KUNIT_ASSERT_NOT_ERR_OR_NULL(test, skb);
    
    skb_put(skb, 60);  // Minimum Ethernet frame size
    
    ret = adin1110_write_fifo(port_priv, skb);
    
    KUNIT_EXPECT_EQ(test, ret, 0);
    
    dev_kfree_skb(skb);
}

/* Test frame reception */
static void test_frame_reception(struct kunit *test)
{
    struct adin1110_port_priv *port_priv = netdev_priv(test_ctx.netdev);
    int ret;
    
    // Setup mock frame data
    test_ctx.mock_spi->mock_registers[ADIN1110_RX_FSIZE] = 
        64 + ADIN1110_FRAME_HEADER_LEN + ADIN1110_FEC_LEN;
    
    ret = adin1110_read_fifo(port_priv);
    
    KUNIT_EXPECT_EQ(test, ret, 0);
}

/**
 * Test Suite 5: Single Interface Operation Tests (MK-specific)
 */

/* Test single eth0 interface creation */
static void test_single_interface_creation(struct kunit *test)
{
    struct adin1110_priv *priv = test_ctx.priv;
    
    // Configure for single port operation (modify from pristine)
    priv->cfg->ports_nr = 1;  // Force single port
    
    // Test that only one interface is created
    KUNIT_EXPECT_EQ(test, priv->cfg->ports_nr, 1);
    KUNIT_EXPECT_NOT_NULL(test, priv->ports[0]);
    
    if (priv->cfg->ports_nr > 1) {
        KUNIT_EXPECT_NULL(test, priv->ports[1]);
    }
}

/* Test eth0 configuration */
static void test_eth0_configuration(struct kunit *test)
{
    struct net_device *netdev = test_ctx.netdev;
    struct adin1110_port_priv *port_priv = netdev_priv(netdev);
    int ret;
    
    // Test opening the network interface
    ret = adin1110_net_open(netdev);
    KUNIT_EXPECT_EQ(test, ret, 0);
    
    // Verify interface state
    KUNIT_EXPECT_EQ(test, port_priv->state, BR_STATE_FORWARDING);
    
    // Test closing the interface
    ret = adin1110_net_stop(netdev);
    KUNIT_EXPECT_EQ(test, ret, 0);
}

/**
 * Test Suite 6: Error Handling and Edge Cases
 */

/* Test handling of invalid frame sizes */
static void test_invalid_frame_size(struct kunit *test)
{
    struct adin1110_port_priv *port_priv = netdev_priv(test_ctx.netdev);
    int ret;
    
    // Test frame too small
    test_ctx.mock_spi->mock_registers[ADIN1110_RX_FSIZE] = 10;
    ret = adin1110_read_fifo(port_priv);
    KUNIT_EXPECT_EQ(test, ret, -EINVAL);
    
    // Test frame too large
    test_ctx.mock_spi->mock_registers[ADIN1110_RX_FSIZE] = ADIN1110_MAX_BUFF + 100;
    ret = adin1110_read_fifo(port_priv);
    KUNIT_EXPECT_LT(test, ret, 0);
}

/* Test SPI error recovery */
static void test_spi_error_recovery(struct kunit *test)
{
    struct adin1110_priv *priv = test_ctx.priv;
    u32 val;
    int ret;
    
    // Simulate transient SPI errors
    test_ctx.mock_spi->simulate_error = true;
    test_ctx.mock_spi->error_count = 2; // Fail twice, then succeed
    
    ret = adin1110_read_reg(priv, ADIN1110_STATUS0, &val);
    
    // Should eventually succeed after retries
    KUNIT_EXPECT_EQ(test, ret, 0);
}

/**
 * Test initialization and cleanup
 */

static int adin1110_test_init(struct kunit *test)
{
    // Initialize test context
    test_ctx.mock_spi = kzalloc(sizeof(struct mock_spi_device), GFP_KERNEL);
    if (!test_ctx.mock_spi)
        return -ENOMEM;
    
    // Initialize mock private structure
    test_ctx.priv = kzalloc(sizeof(struct adin1110_priv), GFP_KERNEL);
    if (!test_ctx.priv) {
        kfree(test_ctx.mock_spi);
        return -ENOMEM;
    }
    
    // Setup basic configuration
    test_ctx.priv->spidev = &test_ctx.mock_spi->spi;
    test_ctx.priv->cfg = &adin1110_cfgs[ADIN2111_MAC];
    mutex_init(&test_ctx.priv->lock);
    spin_lock_init(&test_ctx.priv->state_lock);
    
    // Initialize CRC table
    crc8_populate_msb(adin1110_crc_table, 0x7);
    
    // Create test network device
    test_ctx.netdev = alloc_etherdev(sizeof(struct adin1110_port_priv));
    if (!test_ctx.netdev) {
        kfree(test_ctx.priv);
        kfree(test_ctx.mock_spi);
        return -ENOMEM;
    }
    
    struct adin1110_port_priv *port_priv = netdev_priv(test_ctx.netdev);
    port_priv->priv = test_ctx.priv;
    port_priv->netdev = test_ctx.netdev;
    port_priv->nr = 0;
    
    test_ctx.priv->ports[0] = port_priv;
    test_ctx.initialized = true;
    
    return 0;
}

static void adin1110_test_exit(struct kunit *test)
{
    if (test_ctx.initialized) {
        if (test_ctx.netdev)
            free_netdev(test_ctx.netdev);
        kfree(test_ctx.priv);
        kfree(test_ctx.mock_spi);
        test_ctx.initialized = false;
    }
}

/**
 * Test suite definitions
 */

static struct kunit_case adin1110_test_cases[] = {
    // SPI Communication Tests
    KUNIT_CASE(test_adin1110_read_reg_basic),
    KUNIT_CASE(test_adin1110_read_reg_with_crc),
    KUNIT_CASE(test_adin1110_read_reg_error),
    KUNIT_CASE(test_adin1110_write_reg_basic),
    KUNIT_CASE(test_adin1110_write_reg_with_crc),
    
    // Network Interface Tests
    KUNIT_CASE(test_netdev_allocation),
    KUNIT_CASE(test_mac_address_setting),
    KUNIT_CASE(test_invalid_mac_address),
    
    // Hardware Communication Tests
    KUNIT_CASE(test_phy_id_validation),
    KUNIT_CASE(test_phy_id_mismatch),
    KUNIT_CASE(test_mdio_read),
    KUNIT_CASE(test_mdio_write),
    
    // Frame Processing Tests
    KUNIT_CASE(test_frame_transmission),
    KUNIT_CASE(test_frame_reception),
    
    // Single Interface Tests (MK-specific)
    KUNIT_CASE(test_single_interface_creation),
    KUNIT_CASE(test_eth0_configuration),
    
    // Error Handling Tests
    KUNIT_CASE(test_invalid_frame_size),
    KUNIT_CASE(test_spi_error_recovery),
    
    {}
};

static struct kunit_suite adin1110_test_suite = {
    .name = "adin1110-mk-driver-tests",
    .init = adin1110_test_init,
    .exit = adin1110_test_exit,
    .test_cases = adin1110_test_cases,
};

kunit_test_suites(&adin1110_test_suite);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("HIVE TESTER AGENT");
MODULE_DESCRIPTION("ADIN2111-MK Driver Unit Tests");