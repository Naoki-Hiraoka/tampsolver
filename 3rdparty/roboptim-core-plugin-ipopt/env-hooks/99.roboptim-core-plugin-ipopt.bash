
# setup environment variables for roboptim
export LTDL_LIBRARY_PATH=$LTDL_LIBRARY_PATH:`pkg-config roboptim-core-plugin-ipopt --variable=libdir`/roboptim-core
