# TeslaChargeChallenge

## IDEA:
For this code challenge, My idea is to apply the classic A* searching to find a shortest path and then implement a better charging policy among this path.

In the Astar part, I consider the cost value g as distance between two stations and also the charging rate for current station. It will return a better solution comparing with considering the distance only.

For the charging optimiazation part, the main idea is let the car charges more at the fast rate station, the algorihtm will compare the current charging rate with the next station. If the current station is faster, then the car prefer to charge to full, otherwise it will charge to the amount according to the next moving distance. 


## HOW TO RUN:
```g++ -std=c++11 -O1 main.cpp network.cpp -o candidate_solution```

```./candidate_solution "Albany_NY" "Shreveport_LA"```


## EXP OUTPUT:
```"Albany_NY, Allentown_PA, 1.74863, Laurel_MD, 1.40341, South_Hill_VA, 1.91504, Charlotte_NC, 1.95122, Mountville_SC, 0.478752, Macon_GA, 0.961699, Tifton_GA, 1.72973, DeFuniak_Springs_FL, 2.10164, Mobile_AL, 1.65751, Baton_Rouge_LA, 1.84971, Alexandria_LA, 1.11658, Shreveport_LA"```


## TEST RESULT:
```./checker_linux "Albany_NY, Allentown_PA, 1.74863, Laurel_MD, 1.40341, South_Hill_VA, 1.91504, Charlotte_NC, 1.95122, Mountville_SC, 0.478752, Macon_GA, 0.961699, Tifton_GA, 1.72973, DeFuniak_Springs_FL, 2.10164, Mobile_AL, 1.65751, Baton_Rouge_LA, 1.84971, Alexandria_LA, 1.11658, Shreveport_LA"```
```
Finding Path Between Albany_NY and Shreveport_LA
Reference result: Success, cost was 41.8887
Candidate result: Success, cost was 42.2205
```

## CONCLUSION:
improvements can be:
  - Using MDP to construct the whole problem, and apply dynamic programming for the best solution. 
  - Better charging policy can be considered amoung given path.
  - model the output curve with epslion and CharingParameter to obtain the argmin value. 
