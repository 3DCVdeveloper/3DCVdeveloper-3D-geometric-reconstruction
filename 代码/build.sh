echo "Configuring and building Thirdparty/cpu_tsdf ..."

cd Thirdparty/cpu_tsdf
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j


cd ../../../



echo "Configuring and building orbbec ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
