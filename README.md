CS396 Artificial Life - Jadon Lau
----------------------------------
Here's a GIF teaser of the robot evolution:

![FinalVideo](https://user-images.githubusercontent.com/98376049/224922750-b30eabb9-eacc-45e0-855d-1025cc42850e.gif)


Here's a Youtube link with further information on the robot's evolution:

[https://www.youtube.com/watch?v=uT-AnFQynWU&ab_channel=JadonL.](https://youtu.be/0aDG6SjG670)


Expanding the random creature generator to 3D
----------------------------------
This code creates a 3D creature or animal using rectangular prism and a random number of links/sensor placements. The links are colored either blue or green. The links with the color green have sensors while the blue links are links without sensors. In order to generate to randomly generate a morphology, I chose random sized cubes, sensors, and joints and combined them.

![DiagramMain](https://user-images.githubusercontent.com/98376049/224927469-01c6536a-a6ac-4c70-9adf-1347fd9c1010.jpg)


Changing the random creature for fitness
--------------------------------
I chose to have two options to change the creature for fitness. The first one is the same as it usually was, which was changing the sensor weights. For this assignment, however, I also wanted to change the links placement so that the morphology could change to a better creature as it evolved. I did this by randomly adding/removing links to to robot at every generation and testing their fitness. 

Here's a diagram of a random addition of a link:

![SecondDiagram](https://user-images.githubusercontent.com/98376049/224927495-6badfbab-1d5d-4e15-8eaa-11731d7148b7.jpg)

Better body's fitnesses were saved and the brains were updated everytime mutate was called and another body was generated. Random brains would automatically generated with each random robot. No evolution was done on this part. Robots with better link positions and the correct brain connections would move further and be filtered through to the next generation.

Fitness
--------------------------
Robots that had the best fitness were written in the parallelHillClimber file which measured the robots ability to move along the negative part of the x-axis. Parallel hill climber would allow the simulation to run many robots in parallel and find the most fit robot out of a specific population. That data would then get saved and compared with the next generation. A robot that was able to move further through the x-axis was given a higher fitness function. Higher fitnesses were saved and compared to next generations. The fitness function was used againt 100 generations and a population size of 3. Larger generation and population sizes can be used although it would take much longer. Here are five fitness curves of the best creatures. The x-axis represents generations while the y-axis represents fitness levels. 

![396HW8 fitness charts (1)-1](https://user-images.githubusercontent.com/98376049/224924321-8a982c7b-162e-43b5-963c-7cd1d2954316.jpg)
------------------------------
Running the Code:
------------------------
1.) Clone the repository

2.) Run the command: "pip3 install pybullet" in the terminal to properly install packages to run simulation.

3.) Run the search.py file in the folder

Note: You can change generation and population values in the constant.py folder. You can also see the fitnesses of each generation through the fitnessPlot.txt file. This was used to plot the various fitness of different robots.

-----------------------------
Credit: Much of the code and the information used to make these robots were taken from the ludobots subreddit: https://www.reddit.com/r/ludobots/

I also had help from posts made in our classes Campuswire page. Thank you to the students and TA alike.

This was made with the guidance of Profesor Sam Kriegman and his CS396-Artificial Life class at Northwestern University.
