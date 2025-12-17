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

| Filename                | Kit Source                                           | Status                                          |
|-------------------------|------------------------------------------------------|-------------------------------------------------|
| CANACC4-A501-2Q.json    | Discontinued                                         | Functionally same as CANACC4_2                  |
| CANACC4_2-A508-2N.json  | Discontinued                                         | Tested by Sven Rosvall                          |
| CANACC4_2-A508-2Q.json  | Discontinued                                         | Tested by Sven Rosvall                          |
| CANACC5-A502-2U.json    | Kit Locker Classic                                   | Functionally same as 2V                         |
| CANACC5-A502-2V.json    | Kit Locker Classic                                   | tested by David Ellis                           |
| CANACC8-A503-2V.json    | Discontinued                                         | Uses same code base as CANACC5                  |
| CANACE8C-A505-2n.json   | Kit Locker Classic                                   | Functionally same as 2q                         |
| CANACE8C-A505-2p.json   | Kit Locker Classic                                   | Functionally same as 2q                         |
| CANACE8C-A505-2q.json   | Kit Locker Classic                                   | Tested by Sven Rosvall                          |
| CANBIP-OUT-A535-5b.json | RME UK                                               | Uses same code base as CANACC5                  |
| CANCDU-A524-1c.json     | RME UK                                               | Tested by Neale Brodie                          |
| CANCMD-A50A-4f.json     | MERG Kitlocker                                       | Tested by Simon West                            |
| CANCMDB-A553-4f.json    | In development                                       | Similar to CANCMD                               |
| CANCSD-A537-4d.json     | RME UK                                               | Similar to CANCMD                               |
| CANCSD-A537-4f.json     | RME UK                                               | Similar to CANCMD                               |
| CANINP-A53E-2s.json     | Kit locker basic                                     | Same firmware as CANACE8C                       |
| CANMIO-A520-xx.json     | Universal firmware for CANMIO & CANVxxx kits.        | Tested by Sven Rosvall                          |
| CANMIO-OUT-A534-5b.json | CANMIO-OUT in RME UK; CANVOUT in Kit Locker Advanced | Uses same code base as CANACC5                  |
| CANMIO-SVO-A532-4S.json | MERG Kitlocker CANMIO & CANVSERVO                    | Tested by Sven Rosvall                          |
| CANOUT                  | Kit Locker Basic                                     | Uses same code base as CANACC5                  | 
| CANSCAN                 | RME UK                                               | Develoment of CANACE3C. Tested by Robert Thomas |
| CANSERVO8C-A513-2U.json | CANMIO build option.                                 | Functionally same as 4S                         |
| CANSERVO8C-A513-4h.json | CANMIO build option.                                 | Functionally same as 4S                         |            
| CANSERVO8C-A513-4S.json | CANMIO build option.                                 | Tested by Greg Palmer.                          |
| CANSOL-A522-1B.json     | Kit Locker Basic                                     | Tested by Sven Rosvall                          | 
| CANPAN-A51D-xx.json     | MERG Kitlocker                                       | Tested by Sven Rosvall & Greg Palmer            |
| CANTOTI-A511-2q.json    | Runs on CANACE8C                                     | Same firmware as CANACE8C                       |
| CANXIO-A540-xx.json     | RME UK                                               | Similar to CANMIO with more i/o                 |

## Small Changes Required
The following files are in progress and only small changes are required
to complete them.

| Filename            | Kit Source                                    | Status                                               |
|---------------------|-----------------------------------------------|------------------------------------------------------|

## Unknown but look complete
These files have entries for ```nodeVariables``` and ```eventVariables```
but have not been tested.

| Filename                | Kit Source                                   | Status                 |
|-------------------------|----------------------------------------------|------------------------|
| CAN1IN1OUT-0D63-1a.json | Example included in Arduino CBUS Library.    | |
| CANCSB-A537-4d          | copy of CANCMD-A50A-4d.json                  | |
| CANLED64-A507-2G.json   | Discontinued                                 | |
| CANSERVO-A50B-2H.json   | CANMIO build option. Replaced by CANSERVO8C. | |
| CANSERVO-A50B-3H.json   | CANMIO build option. Replaced by CANSERVO8C. | |
| CANSLOT-0D03-1a.json    | ??                                           | |
| SER1IN1OUT-FAFE-1a.json | Example included in Arduino CBUS Library.    | |

## Files without ```eventVariables```
These files have been created but event variables have not been added yet.

| Filename                | Kit Source                                           | Status                      |
|-------------------------|------------------------------------------------------|-----------------------------|
| CANACE3-A504-2G.json    | Discontinued                                         |                             |
| CANACE3C-A51E-3a.json   | Discontinued                                         |                             |
| CANACE8MIO-A521-2q.json | CANMIO build option.                                 |                             |
| CANLEVER-0D20-1a.json   | ??                                                   |                             |
| CANSERVO-A50B-2u.json   | CANMIO build option.                                 | Older version of CANSERVO8C |

## Missing files
The following modules are available in the MERG Kitlocker or from RME UK but 
do not yet have any Module Descriptor File.

| Module   | Kit Source          | Status                         |
|----------|---------------------|--------------------------------|
| CANVINP  | Kit Locker Advanced | Works with Universal firmware. | 
| CANV4BIP | Kit Locker Advanced | Works with Universal firmware. |
| CANDISP  | RME UK              | Development of CANLED64        |

There are more module firmwares describe in the MERG Knowledgebase.
These will be added to the wish list upon request.
