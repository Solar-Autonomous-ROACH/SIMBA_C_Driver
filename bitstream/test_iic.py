from pynq import Overlay
from pynq.lib.iic import AxiIIC

overlay = Overlay("Kria_BD_wrapper.bit")
iic = AxiIIC(overlay.ip_dict["axi_iic_0"])

dev_addr = 0x69
reg = 0x0

bytes_to_rx = 1
rx_data = bytes(bytes_to_rx)

tx_data = [0x00]

print("Send data...")
iic.send(dev_addr, tx_data, len(tx_data), 1)
print("Request receive...")
iic.receive(dev_addr, rx_data, bytes_to_rx)
print("Waiting...")
iic.wait()
print("Register[" + str(hex(reg)) + "] value is: " + str(rx_data))
