if [ ! -d "build" ]; then
    mkdir -p build
fi

cd ./build

rm -rf *

cmake ../ 

make -j8

# scp ./K253154 root@192.168.20.100:/userdata