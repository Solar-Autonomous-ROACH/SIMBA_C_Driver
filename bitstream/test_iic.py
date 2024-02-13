from pynq import Overlay
from pynq.lib.iic import AxiIIC

overlay = Overlay("Kria_BD_wrapper.bit")
iic = AxiIIC(overlay.ip_dict["axi_iix_0"])

dev_addr = 0x68
for x in range(255):
    iic_data = [x]
    tx_data = bytes(1)
    iic.send(dev_addr, iic_data, len(iic_data), 1)
    iic.receive(dev_addr, tx_data, 1, 0)
    iic.wait()
    print("Register[" + str(hex(x)) + "] value is: " + str(tx_data))
