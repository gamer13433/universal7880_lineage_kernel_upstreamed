/*
 * OF helpers for usb devices.
 *
 * This file is released under the GPLv2
 */

#ifndef __LINUX_USB_OF_H
#define __LINUX_USB_OF_H

#include <linux/usb/ch9.h>
#include <linux/usb/otg.h>
#include <linux/usb/phy.h>

#if IS_ENABLED(CONFIG_OF)
enum usb_dr_mode of_usb_get_dr_mode(struct device_node *np);
enum usb_device_speed of_usb_get_maximum_speed(struct device_node *np);
bool of_usb_host_tpl_support(struct device_node *np);
<<<<<<< HEAD
unsigned int of_usb_get_suspend_clk_freq(struct device_node *np);
=======
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
#else
static inline enum usb_dr_mode of_usb_get_dr_mode(struct device_node *np)
{
	return USB_DR_MODE_UNKNOWN;
}

static inline enum usb_device_speed
of_usb_get_maximum_speed(struct device_node *np)
{
	return USB_SPEED_UNKNOWN;
}
<<<<<<< HEAD

=======
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
static inline bool of_usb_host_tpl_support(struct device_node *np)
{
	return false;
}
<<<<<<< HEAD

static inline unsigned int of_usb_get_suspend_clk_freq(struct device_node *np)
{
	return 0;
}
=======
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
#endif

#if IS_ENABLED(CONFIG_OF) && IS_ENABLED(CONFIG_USB_SUPPORT)
enum usb_phy_interface of_usb_get_phy_mode(struct device_node *np);
#else
static inline enum usb_phy_interface of_usb_get_phy_mode(struct device_node *np)
{
	return USBPHY_INTERFACE_MODE_UNKNOWN;
}

#endif

#endif /* __LINUX_USB_OF_H */
