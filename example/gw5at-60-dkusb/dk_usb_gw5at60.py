"""Compatibility shim -- the DK_USB platform moved into the package.

The board platform is now ``gowin_serdes.dkusb_gw5at60`` so that
out-of-tree consumers (the luna-ss fork's examples, the gw_usb3 test
suite) can import it without sys.path reaches.  This shim keeps the
in-tree examples' historical ``from dk_usb_gw5at60 import ...`` alive.
"""

from gowin_serdes.dkusb_gw5at60 import *          # noqa: F401,F403
from gowin_serdes.dkusb_gw5at60 import (          # noqa: F401
    DKUSBGW5AT60Platform, SERDES_PINS, add_serdes_refclk_forward)
