CS396 Artificial Life - Jadon Lau
----------------------------------
Running the Code:

1.) Clone the repository

2.) Run the search.py file in the folder

Expanding the random creature generator to 3D
----------------------------------
This code creates a 3D creature or animal using rectangular prism and a random number of links/sensor placements. The links are colored either blue or green. The links with the color green have sensors while the blue links are links without sensors. In order to generate to randomly generate a random morphology, I needed to 

Fitness
--------------------------
Robots that had the best fitness were written in the parallelHillClimber file which measured the robots ability to move along the negative part of the x-axis. Higher fitnesses were saved and compared to next generations.


![Untitled Notebook 5-1](https://user-images.githubusercontent.com/98376049/220240861-147931a1-0cf9-4b29-b33d-3ce197868b93.png)

-----------------------------
Credit: Much of the code and the information used to make these robots were taken from the ludobots subreddit: https://www.reddit.com/r/ludobots/
