# Dev notes

* CubeIDE recomends user to enable the `ICACHE` for maximum perfomance.
* OpenOCD does not yet support the H5, [ST maintains a fork with support](https://github.com/STMicroelectronics/OpenOCD).
* OpenOCD does not come with udev rules for the new ST-Link V3, here's the required line:
```
# STLink v3
ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374e", MODE="660", GROUP="plugdev", TAG+="uaccess"
```
