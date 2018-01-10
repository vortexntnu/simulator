mkdir build
cd build
cmake ..
make

mv libsimple_ROV_plugin.so $PWD/../model/rov/plugins/
