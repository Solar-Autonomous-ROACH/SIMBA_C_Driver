INSTRUCTIONS

To run the blink example:

LOAD THE BITSTREAM
sudo su -
cd /home/ubuntu/SIMBA_C_Driver/bitstream
python3 load.py
exit

COMPILE THE C CODE
cd ~/SIMBA_C_Driver/test
make

RUN THE C CODE
cd ~/SIMBA_C_Driver/test
sudo ./test
