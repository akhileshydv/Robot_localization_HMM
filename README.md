# Robot_localization_HMM
The objective of this is to apply Hidden Markov Models to localization problem.
Consider a robot with the task of localization (inferring where it is from the available data) given a map
of the world and a sequence of percepts and actions. The robot is placed in a maze-like environment as
shown in Figure.
![screenshot from 2018-07-08 11-51-44](https://user-images.githubusercontent.com/8543981/42417357-8dbba8ba-82a5-11e8-8a0d-a7b296368637.png)

The robot is equipped with four sonar sensors that tell whether there is an obstacle- the outer wall or a
shaded square in the figure – in each of the compass directions (NSEW). We assume that the robot has a
correct map. The robot performs action _Move_ to move to one of the adjacent or neighbouring square.

_X_<sub>t</sub>: state variable representing the location of the robot on the discrete grid.

_dom_(_X_<sub>t</sub>) = {s<sub>1</sub>,...,s<sub>n</sub>}: domain of _X_<sub>t</sub> the set of empty squares.

NEIGHBOURS(s): the set of empty squares that are adjacent to and let N(s) be the size of that set.

The transition model for _Move_ action is given as:
![image](https://user-images.githubusercontent.com/8543981/42417836-0b631bc6-82b1-11e8-8dc5-2dccf0212622.png)

Assume uniform distribution over all the squares; P(_X_<sub>o</sub>= i)= 1/_n_.

_E_<sub>t</sub>: sensor variable that can have 16 possible values, each a four-bit sequence giving the presence or
absence of an obstacle in a particular compass direction.

The observation model can be given as:
![screenshot from 2018-07-08 12-31-24](https://user-images.githubusercontent.com/8543981/42417563-3fbad07c-82ab-11e8-8e3c-7d6a24f9081d.png)


&epsilon; is the sensor’s error rate and errors occur independently for the four sensor directions, (1- &epsilon;)<sup>t</sup> is 
probability of getting all the four bits right and the probability of getting all of them wrong is &epsilon;<sup>d<sub>it</sub></sup>, and d<sub>it</sub> is the discrepancy that is the number of bits that are different – between true values for square _i_ and
the actual reading e<sub>t</sub> from the sensor.

(a) First the robot needs to estimate its current location and then determine the most likely path it has
taken to get where it is now for a given time _t_.

(b) Repeat (a) and find out the localization error as a function of number of observations for various
values of (0.00, 0.02, 0.05, 0.10, 0.20) and plot them. Localization error is Manhattan distance from
the true location.

(c) Repeat (b) for path accuracy, which is defined as the fraction of correct states on the Viterbi path.

## Plots:
![figure_1-2](https://user-images.githubusercontent.com/8543981/42417876-b4bbb8cc-82b1-11e8-88fd-e6f5ec497f83.png)
![figure_2-2](https://user-images.githubusercontent.com/8543981/42417877-b567e4d0-82b1-11e8-8a7b-3b0336913aac.png)
