# PXRF
```
sudo apt-get install qt5-default
sudo apt-get install libqt5websockets5-dev
```

Need following Kernel driver: \
CONFIG_USB_NET_CDC_EEM=y

## PXRF Data Folder:

Generates a results folder in the home directory if it doesn't already exist.

~/pxrf_results

## CSV File:

The CSV follows the naming convention below.

YYYY-MM-DD.csv

A new CSV is generated if and only if a scan from the calendar day has not been taken yet.
