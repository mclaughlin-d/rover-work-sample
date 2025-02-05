# Autonomous Typing Project (in-progress)

I am actually working on this project currently (doing some testing and fine-tuning), but I included
it as an example of work I have done that interfaces with an IK solver.

## Description
This code serves as a first test implementation of autonomous typing capability. It works with the assumption
that the arm starts up lined up with some known key, and uses an IK solver to move to/from each key in a sequence of 
characters, which can be set by publishing to certain topics.