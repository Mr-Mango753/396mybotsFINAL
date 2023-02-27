CS396 Artificial Life - Jadon Lau
----------------------------------
Running the Code:

1.) Clone the repository

2.) Run the search.py file in the folder

Expanding the random creature generator to 3D
----------------------------------
This code creates a 3D creature or animal using rectangular prism and a random number of links/sensor placements. The links are colored either blue or green. The links with the color green have sensors while the blue links are links without sensors. In order to generate to randomly generate a morphology, I chose random sized cubes, sensors, and joints and combined them.

![IMG_0309](https://user-images.githubusercontent.com/98376049/221701647-4ab2cfee-9ffc-4139-b18b-0bbec4c2b111.jpg)

Changing the random creature for fitness
--------------------------------
I chose to have two options to change the creature for fitness. The first one is the same as it usuallyw as, which was changing the sensor weights. For this assignment, however, I also wanted to change the links placement so that the morphology could change to a better creature as it evolved. I did this by randomly adding links to to robot at every generation and testing their fitness.

Fitness
--------------------------
Robots that had the best fitness were written in the parallelHillClimber file which measured the robots ability to move along the negative part of the x-axis. Higher fitnesses were saved and compared to next generations. The fitness function was used againt 100 generations and a population size of 3. Larger generations and populations can/should be used for computer with higher processing power. Here are five fitness curves of the best creatures.

[396HW8 fitness charts.pdf](https://github.com/Mr-Mango753/396mybotsHW8/files/10844856/396HW8.fitness.charts.pdf)


-----------------------------
Credit: Much of the code and the information used to make these robots were taken from the ludobots subreddit: https://www.reddit.com/r/ludobots/
