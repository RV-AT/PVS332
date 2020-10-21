#!/usr/bin/tcsh -f
#-------------------------------------------
# qflow exec script for project /mnt/d/Projects/FPGA/PVS332/flow
#-------------------------------------------

# /usr/lib/qflow/scripts/synthesize.sh /mnt/d/Projects/FPGA/PVS332/flow prv332sv0 || exit 1
/usr/lib/qflow/scripts/placement.sh -d /mnt/d/Projects/FPGA/PVS332/flow prv332sv0 || exit 1
# /usr/lib/qflow/scripts/vesta.sh /mnt/d/Projects/FPGA/PVS332/flow prv332sv0 || exit 1
# /usr/lib/qflow/scripts/router.sh /mnt/d/Projects/FPGA/PVS332/flow prv332sv0 || exit 1
# /usr/lib/qflow/scripts/placement.sh -f -d /mnt/d/Projects/FPGA/PVS332/flow prv332sv0 || exit 1
# /usr/lib/qflow/scripts/router.sh /mnt/d/Projects/FPGA/PVS332/flow prv332sv0 || exit 1 $status
# /usr/lib/qflow/scripts/cleanup.sh /mnt/d/Projects/FPGA/PVS332/flow prv332sv0 || exit 1
# /usr/lib/qflow/scripts/display.sh /mnt/d/Projects/FPGA/PVS332/flow prv332sv0 || exit 1
