# Status of Module Descriptor Files
Module Descriptor Files are either kept in the directory ```production```
for files that are complete and tested, or in the directory ```work_in_progress```
for files that still need work.
The current state of the work_in_progress files are described below.

Also check submitted [Issues](https://github.com/david284/ModuleDescriptor/issues)
to the ModuleDescriptor repository.

## Ready for Production
The following files are complete and need a final review before moving to
the production directory.

| Filename                | Kit Source                        | Status                          |
|-------------------------|-----------------------------------|---------------------------------|
| CANCMD-A50A-4d.json     | MERG Kitlocker                    | Tested by Sven Rosvall          |
| CANMIO-A520-xx.json     | Universal firmware for CANMIO & CANVxxx kits. | Tested by Sven Rosvall          |
| CANMIO-SVO-A532-4s.json | MERG Kitlocker CANMIO & CANVSERVO | Tested by Sven Rosvall          |
| CANPAN-A51D-1y.json     | MERG Kitlocker                    | Tested by Sven Rosvall          |
| CANXIO-A540-xx.json     | RME UK                            | Similar to CANMIO with more i/o |

## Small Changes Required
The following files are in progress and only small changes are required
to complete them.

| Filename                | Kit Source                                    | Status                                               |
|-------------------------|-----------------------------------------------|------------------------------------------------------|
| CANSCAN-A531-4a101.json | Ian Hart                                      | Created & tested by Richard Crawshaw                 |
| CANSCAN-A531-4d.json    | Ian Hart                                      | Created & tested by Richard Crawshaw                 |


## Unknown but look complete
These files have entries for ```nodeVariables``` and ```eventVariables```
but have not been tested.

| Filename            | Kit Source                                  | Status                 |
|---------------------|---------------------------------------------|------------------------|
| CAN1IN1OUT-0D63-1a.json | Example included in Arduino CBUS Library.   | |
| CANACC4-A501-2q.json | Discontinued                                | |
| CANACC4_2-A508-2q.json | Discontinued                                | |
| CANACC5-A502-2V.json | Kit Locker Classic                          | |
| CANACC5-A502-2u.json | Kit Locker Classic                          | |
| CANACC8-A503-2v.json | Discontinued                                | |
| CANACE8C-A505-2q.json | Kit Locker Classic                          | |
| CANLED64-A507-2g.json | Discontinued                                | |
| CANSERVO-A50B-2H.json | CANMIO build option. Replaced by CANSERVO8C. | |
| CANSERVO-A50B-3h.json | CANMIO build option. Replaced by CANSERVO8C. | |
| CANSERVO8C-A513-2u.json | CANMIO build option.                        | |
| CANSERVO8C-A513-4h.json | CANMIO build option.                        | |
| CANSLOT-0D03-1a.json | ??                                          | |
| CANTOTI-A511-2q.json | Runs on CANACE8C                            | |
| SER1IN1OUT-FAFE-1a.json |                                             | |

## Files without ```eventVariables```
These files have been created but event variables have not been added yet.

| Filename            | Kit Source                                           | Status                 |
|---------------------|------------------------------------------------------|------------------------|
| CANACE3-A504-2g.json | Discontinued                                         | |
| CANACE3C-A51E-3a.json | Discontinued                                         | |
| CANACE8MIO-A521-2q.json | CANMIO build option.                                 | |
| CANBIP-OUT-A535-5b.json | RME UK                                               | |
| CANINP-A53E-2s.json | Kit locker basic                                     | |
| CANLEVER-0D20-1a.json | ??                                                   | |
| CANMIO-OUT-A534-5b.json | CANMIO-OUT in RME UK; CANVOUT in Kit Locker Advanced | |
| CANSERVO-A50B-2u.json | CANMIO build option.                                 | |

## Missing files
The following modules are available in the MERG Kitlocker or from RME UK but 
do not yet have any Module Descriptor File.

| Module   | Kit Source          | Status                         |
|----------|---------------------|--------------------------------|
| CANOUT   | Kit Locker Basic    |                                | 
| CANSOL   | Kit Locker Basic    |                                | 
| CANVINP  | Kit Locker Advanced | Works with Universal firmware. | 
| CANV4BIP | Kit Locker Advanced | Works with Universal firmware. |
| CANCDU   | RME UK              |                                |

There are more module firmwares describe in the MERG Knowledgebase.
These will be added to the wish list upon request.
