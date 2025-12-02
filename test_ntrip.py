#!/usr/bin/env python3
from logging import getLogger
from time import sleep
from pygnssutils import VERBOSITY_HIGH, GNSSNTRIPClient, set_logging

logger = getLogger("pygnssutils.gnssntripclient")
set_logging(logger, VERBOSITY_HIGH)

with open("test_rtcm.log", "wb") as out:
    gnc = GNSSNTRIPClient()
    
    print("Starting NTRIP client...")
    gnc.run(
        server="rtk2go.com",
        port=2101,
        https=0,
        mountpoint="Renewables",
        datatype="RTCM",
        ntripuser="s229701698-at-mandela.ac.za",
        ntrippassword="mukandagumbo",
        ggainterval=10,
        ggamode=1,
        reflat=-33.9608,
        reflon=25.6022,
        refalt=50.0,
        refsep=0.0,
        output=out,
    )
    
    try:
        for i in range(60):  # Wait 60 seconds
            sleep(1)
            print(f"Waiting... {i+1}s")
    except KeyboardInterrupt:
        print("Stopped")
