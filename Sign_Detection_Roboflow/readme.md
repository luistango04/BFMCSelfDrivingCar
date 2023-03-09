sudo apt update

wget https://curl.se/download/curl-7.55.1.tar.gz
./configure --with-openssl
make
make test 
sudo make install

pip3 install numpy==1.19.5
sudo apt install libjpeg-dev
pip3 install matplotlib
sudo apt install gfortran
sudo apt install libopenblas-dev
sudo apt install liblapack-dev
pip3 install scipy
pip3 install typing-extensions
pip3 install requests

sudo nvpmodel -m 0
sudo jetson_clocks

#You need internet connection for the first time to download the trained data. 
#Install the inference server

sudo docker pull roboflow/inference-server:jetson

#Run the infernce server
sudo docker run --net=host --gpus all roboflow/inference-server:jetson

#Run the code with intel realsense or without

python3 infer_local_with_realsense.py

#OR 

python3 infer_local_without_realsense.py
