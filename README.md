# ModuleDescriptor
VLCB/CBUS Module descriptor format &amp; examples.

This format aims to provide a way to describe a module in a form that can be digested by a computer program to provide a meaningful user interface, without changing the software programming

# Documents
Please see the latest [Module Descriptor File Format document](documents) for detailed information.

# File naming
The naming of the module descriptors follows the following format:

```<module name>-<module identity>-<version>.json```

Where the module identity is the combination of the manufacturer & module id in hex.
And the version number is the major & minor number as reported by the module.

An example for the CANMIO module is CANMIO-A520-3d.json

Note that for development work, a manufacturer code of 13 is recommended (0x0D), 
allowing any module id to be used for an individuals own use.

# work_in_progress
This folder contains module descriptors that whilst are valid files, are likely to be incomplete, 
and not yet validated as complete.

See the [Status](Status.md) page for information about current status for
each descriptor file and what work is left to complete them.  
This folder is sub-divided as follows...  
#### MERG modules
MERG modules have manufacturer ID `MERG` & module numbers assigned by them
#### Development modules
Development modules use the manufacturer code '13' as assigned by NMRA to `Public Domain & Do-It-Yourself Decoders`, and the best choice for anyone initially developing a new module  
It's free to use any module number, so allowing for multiple different projects, but it is the users responsibility to ensure the module number is suitable and won't conflict on their system. 
So files here may need their module ID changed to suit
#### Test modules
These are intended for 'in-house' testing, not for general use

# generators
This folder contains scripts for some modules that will generate a module descriptor file for 
the appropriate module.

A generator is useful where there is repeated information, e.g. the same content for channels 
1 to 8, with tokens for simple substitution like variable number.

Please see the [README](generators/README.md) in the generator folder for more information.

# Committing Changes

Before committing any changes you must set up Git hooks.
See the specific [README](git-hooks/README.md) that describes this.

Module descriptor files contain a "timestamp" element that is used to
check if a user provided descriptor file is older than a system descriptor 
file.
It is important to update this "timestamp" element whenever a descriptor file
is updated.

A Git hook is provided to automatically update the "timestamp" element
when updates to descriptor files are committed.
The README file above describes how this hook is set up.
