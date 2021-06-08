#! /bin/bash
sudo apt-get update
sudo apt-get install -y libuv1-dev libssl-dev libz-dev
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ..
cd ..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets

# install simulator
mkdir simulator
cd simulator
wget https://github.com/udacity/self-driving-car-sim/releases/download/T3_v1.2/term3_sim_linux.zip
unzip term3_sim_linux.zip
cd term3_sim_linux
chmod +x term3_sim.$(uname -m)
cd ..
cd ..
