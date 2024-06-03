# Path-Planning-Algorithms-Comparison
A study and implementation of RRT variants
# Optimizing Robotic Path Planning with Enhanced RRT Variants

## Project Overview

This project focuses on enhancing the efficiency of path-planning algorithms, specifically Rapidly-exploring Random Trees (RRT) and its variants. Path planning is a fundamental problem in robotics and artificial intelligence, with applications ranging from autonomous navigation to industrial automation. This project aims to compare and improve various RRT variants to determine the best algorithm for different environmental conditions.

## Team Members

- Prathinav Karnala Venkata
  - M.Eng Robotics, University of Maryland, College Park, Maryland, USA
  - Email: pratkv@umd.edu

- Pranav ANV
  - M.Eng Robotics, University of Maryland, College Park, Maryland, USA
  - Email: anvpran@umd.edu

- Sarang Shibu
  - Robotics Program, University of Maryland, College Park, Maryland, USA
  - Email: sarang@umd.edu

## Algorithms Analyzed

- **RRT (Rapidly-exploring Random Tree)**: A basic algorithm designed for efficiently exploring high-dimensional spaces.
- **RRT-Connect**: Enhances RRT by growing two trees simultaneously from the start and goal points.
- **Informed RRT***: Improves RRT* by narrowing the sampling space to the most promising region.
- **EP-RRT***: Combines the targeted sampling of Informed RRT* with the rapid exploration of RRT-Connect.

## Key Contributions

- **Comparison and Analysis**: Evaluated the efficiency, computation time, solution quality, and scalability of various RRT variants.
- **Modified EP-RRT***: Developed a modified version of EP-RRT* incorporating greedy exploitation and sampling efficiency techniques for improved path planning.

## Results

- **Time and Space Complexities**: Detailed time and space complexities for RRT, RRT-Connect, Informed RRT*, and EP-RRT*.
- **Simulation Performance**: Conducted multiple simulation test cases demonstrating the efficiency and effectiveness of the modified EP-RRT* algorithm.

## Methodology

The project implemented a combination of greedy exploitation from RRT-Connect and sampling efficiency from Informed RRT* to develop a modified EP-RRT* algorithm. The methodology involved evaluating the algorithms' performance using various start and goal coordinates, step sizes, and iterations to ensure robust results.

## Applications

The findings of this project are applicable to real-world scenarios such as autonomous navigation, industrial automation, and robotic agriculture, where efficient and reliable path planning is crucial.

## Conclusion

The project successfully demonstrated the potential of RRT variants in robotic path planning and identified challenges such as parameter sensitivity, handling complex obstacles, algorithmic convergence, and computational efficiency. The modified EP-RRT* algorithm showed significant improvements in efficiency and effectiveness.


