database -open waves -shm -into ../out/waves.shm
probe -create -database waves test_bench -shm -all -depth all
run
quit
