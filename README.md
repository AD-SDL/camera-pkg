# RPL - Camera Vision protocols

A repository for CV related tasks at RPL

## Camera Clients

Python interface that allows remote commands to be executed 

- Cameras are used for different operations / control / checks at RPL.

### Current features
* Camera initialization..
* Detect Wells
* Detect Pipetes
* read QR

## Install

    conda create -n rpl-cv-test python=3.8
    conda activate rpl-cv-test

    git clone https://github.com/AD-SDL/rpl-camera-driver.git
    cd rpl-camera-driver
    pip install -r requirements.txt
    pip install -e . 

Better to install in develop-mode while the config is still changing


## Ros Install

