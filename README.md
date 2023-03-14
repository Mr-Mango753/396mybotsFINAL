CS396 Artificial Life - Jadon Lau
----------------------------------
Here's a GIF teaser of the robot evolution:

![FinalVideo](https://user-images.githubusercontent.com/98376049/224922750-b30eabb9-eacc-45e0-855d-1025cc42850e.gif)


Here's a Youtube link with further information on the robot's evolution:

Expanding the random creature generator to 3D
----------------------------------
This code creates a 3D creature or animal using rectangular prism and a random number of links/sensor placements. The links are colored either blue or green. The links with the color green have sensors while the blue links are links without sensors. In order to generate to randomly generate a morphology, I chose random sized cubes, sensors, and joints and combined them.

![IMG_0309](https://user-images.githubusercontent.com/98376049/221701647-4ab2cfee-9ffc-4139-b18b-0bbec4c2b111.jpg)

Changing the random creature for fitness
--------------------------------
I chose to have two options to change the creature for fitness. The first one is the same as it usually was, which was changing the sensor weights. For this assignment, however, I also wanted to change the links placement so that the morphology could change to a better creature as it evolved. I did this by randomly adding links to to robot at every generation and testing their fitness. 

Here's a diagram of a random addition of a link:

Better body's fitnesses were saved and the brains were updated everytime mutate was called and another body was generated. Random brains would automatically generated with each random robot. No evolution was done on this part. Robots with better link positions and the correct brain connections would move further and be filtered through to the next generation.

Fitness
--------------------------
Robots that had the best fitness were written in the parallelHillClimber file which measured the robots ability to move along the negative part of the x-axis. Parallel hill climber would allow the simulation to run many robots in parallel and find the most fit robot out of a specific population. That data would then get saved and compared with the next generation. A robot that was able to move further through the x-axis was given a higher fitness function. Higher fitnesses were saved and compared to next generations. The fitness function was used againt 100 generations and a population size of 3. Larger generations and populations can/should be used for computer with higher processing power. Here are five fitness curves of the best creatures. The x-axis represents generation while the y-axis represents fitness levels. 

[396HW8 fitness charts.pdf](https://github.com/Mr-Mango753/396mybotsHW8/files/10844856/396HW8.fitness.charts.pdf)
------------------------------
Running the Code:

1.) Clone the repository
2.) Run the search.py file in the folder

-----------------------------
Credit: Much of the code and the information used to make these robots were taken from the ludobots subreddit: https://www.reddit.com/r/ludobots/
I also had much help thanks to posts made in Campuswire by TA's and students alike.
