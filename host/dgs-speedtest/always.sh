#!/bin/bash

until cargo run --release
do
    echo "Waiting..."
    sleep 1
done
