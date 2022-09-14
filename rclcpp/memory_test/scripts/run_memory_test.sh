echo 
echo "Fastrtps"
echo 
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ./memory-test
echo 
echo "Cyclone"
echo 
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ./memory-test
echo 
echo "STUB"
echo 
RMW_IMPLEMENTATION=rmw_stub_cpp ./memory-test
echo 
