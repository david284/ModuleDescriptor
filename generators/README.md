
# Generators
This folder contains scripts for some modules that will generate a module descriptor file for the appropriate module.
A generator is useful where there is repeated information, e.g. the same content for channels 1 to 8, with tokens for simple substitution like variable number.

# Requirements
The generators are shell and python scripts.
Older generators uses shell scripts.
They are being converted to Python.

## Python Scripts
You need Python installed on your system to run these generators. 
Python is generally available on most Linux systems and easy to 
install on other systems.

## Shell Scripts
To run them you need ```bash``` which is provided with Linux systems.
For Windows systems ```bash``` can be provided by [Cygwin](https://www.cygwin.com) and [Git Bash](https://git-scm.com/).

# Generating Module Descriptor Files
The generator scripts produce the MDF content as the standard output.
When running the generators redirect this output to the desired target file.
E.g. :

```sh generators/generate_CANPAN.sh > output/CANPAN-0D20-9z.json```

# Writing Generator Scripts

See existing generator scripts for ideas and solutions to tricky needs.

## Coding style
We try to write generator scripts that generate JSON code that is as human readable as
possible.
I.e. using indentation and reasonable spacing.

We also try to write the generators with a similar structure to the generated
JSON file. 
This makes the generator look like a template where variables are used as placeholders
for items to be filled in with variable content.
This style makes it easy to find the correct place in the generator script to fix 
any problems detected in the generated JSON file.

## Python generators
The Python generators build up a datastructure of dictionaries and lists that
make up the resulting MDF contents. 

Conditional and repeating contents are handled with dictionary and list comprehension.
This means that any conditions and loop constructs are shown below the
data elements rather than before as is done with conditional and looping code.

## Shell script generators
Most of the generators are built around "Here documents" which allow expansion of
variables within a text block. 
A "Here document" is an embedded piece of text that is fed as standard input to
a process.
Here we use a ```cat``` command to simply pass that text to the standard output.
An example:
```
cat << EOF
Some text.
EOF
```
Some blocks of text is repeated, for example for each input channel.
For these cases we use a loop that then processes one "Here document" for each input channel.
Example:
```
for channel in 1 2 3 4 5 6 7 8
do
  cat << EOF
Stuff for channel ${channel}.
EOF
done
```

### Comma separated lists in shell scripts
JSON is strict on commas between items in a list and there may not be any trailing comma.
There are many ways to solve this. 
The method used here is to use a function that takes a condition (same syntax as
the builtin [] command) and returns a comma if this condition is true
and returns nothing if the condition is false. This needs to be executed in
a command substitution construct like this example:
```
cat <<EOF
"elements" : [
EOF
for channel in 1 2 3 4
do 
  cat << EOF
   { 
     ...
   }$(commaIf $channel != 4)
EOF
done
cat <<EOF
]
EOF
```
