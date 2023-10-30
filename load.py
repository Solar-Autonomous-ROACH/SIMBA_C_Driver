from pynq import Overlay

ol = Overlay("bitstream/Kria_BD_wrapper.bit")
ol.download()
