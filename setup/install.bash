#!/usr/bin/env bash
# This script installs the Polhemus Patriot tracker udev rules and firmware

sudo apt-get install libusb-1.0 fxload
sudo mkdir /usr/local/share/PolhemusUsb
sudo cp usbfw/PatriotUSB2.hex /usr/local/share/PolhemusUsb/
sudo cp usbfw/a3load.hex /usr/local/share/PolhemusUsb/
sudo cp fw.rules/90-Patriot_trkr.rules /etc/udev/rules.d/
