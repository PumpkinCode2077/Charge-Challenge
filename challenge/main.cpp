#include "network.h"
#include <cmath>
#include <queue>
#include <list>
#include <map>
#include <algorithm>
#include <iterator>

#define pi 3.14159265358979323846
#define radias 6356.752*1000
#define rad2deg pi/180
#define deg2rad 180/pi
#define epsilon 1 //Astar heuristic weight parameter
#define CP 200.0*100 //Astar charging cost scalar
#define pow(x) ((x)*(x))
#define MAXBT 320.

using namespace std;

double cal_distance(const row &place1, const row &place2){
//The funciton for given two stations' info, return the norm2 distance
    double lat_1 = place1.lat * rad2deg;
    double lat_2 = place2.lat * rad2deg;
    double lat_diff = (place2.lat-place1.lat) * rad2deg;
    double lon_diff = (place2.lon-place1.lon) * rad2deg;
    double alpha = pow(sin(lat_diff/2)) + cos(lat_1)*cos(lat_2) * pow(sin(lon_diff/2));
    double beta  = 2 * atan2(sqrt(alpha),sqrt(1-alpha));
    return radias * beta/1000;
};

struct Node{
//Astar node
    double g;
    double f;
    Node *parent;
    row station_info;  
};

struct Node_comp{bool operator() (const Node* a, const Node* b) const{return a->f > b->f;}};

double get_heuristic(const row &place1, const row &place2){
    return cal_distance(place1,place2)*epsilon;
};

void print_path(Node* end_node){
//The function is for given the Astar shortest path, return the optiaml charging police and print path output
//The smart chargeing policy will depends on the current charing rate and next station charging rate
    Node* temp = end_node;
    std::list<Node*> path_list{};
    if(end_node==0){cout<< "Path is empty!" << endl; return;}
    double rate_avg=0.0;
    int station_num = 0;
    while(temp != 0){
        path_list.push_front(temp);
        rate_avg += temp->station_info.rate;
        station_num ++ ; 
        temp = temp->parent;
    }
    double current_battery = MAXBT;
    double charge_time = 0.0;
    Node* last_station = *path_list.begin();
    cout << "\"";

    for(auto i = path_list.begin(); i!= path_list.end(); ++i){
        if(i!=path_list.begin()){(cout<<", ");}
        current_battery = current_battery - cal_distance(last_station->station_info,(*i)->station_info);
        cout << (*i)->station_info.name;
        if (i==prev(path_list.end(),2)){
            //Avoid overcharge at second last station 
            auto j = next(i,1);
            double next_distance = cal_distance((*j)->station_info,(*i)->station_info);          
            charge_time = cal_distance((*j)->station_info,(*i)->station_info) / (*i)->station_info.rate;
            }
        else{
            //Smart chrage, if current station rate is faster then charge to full, otherwise charge to the desired amount
            if((*i)!=end_node){
                auto j = next(i,1);
                double next_distance = cal_distance((*j)->station_info,(*i)->station_info);
                if((*i)->station_info.rate > (*j)->station_info.rate){
                    charge_time = (320-current_battery)/(*i)->station_info.rate;
                    current_battery = 320;
                }
                else{
                    charge_time = (next_distance-current_battery)/(*i)->station_info.rate;
                    current_battery = next_distance;
                }
            }        
        }
        if((*i)!=end_node & i!=path_list.begin()){cout << ", " << charge_time;}
        last_station = (*i);
    }
    cout << "\"" << endl;
}

int main(int argc, char** argv)
//The main algorithm to apply the Astar to find the shortest path from A->B
{
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;        
        return -1;
    }
    std::string initial_charger_name = argv[1];
    std::string goal_charger_name = argv[2];

    int start_idx, goal_idx; 
    for(int i=0; i<network.size();i++){
        if (network[i].name == initial_charger_name) {start_idx=i;};
    	if (network[i].name == goal_charger_name)    {goal_idx =i;};
    }
    //Astar search initialization
    priority_queue<Node*, vector<Node*>, Node_comp> open_list;
    std::vector<Node*> close_list = {};
    std::vector<int>   visited_list = {};
    std::map<int, Node*> node_map;
    Node *start_node; //define a start node
    Node *children_node; //define the childrens node
    start_node = new Node;
    double g = 0.0;
    double h = get_heuristic(network[start_idx],network[goal_idx]);
    start_node->g = g;
    start_node->f = g+h;
    start_node->parent = 0;
    start_node->station_info = network[start_idx];
    open_list.push(start_node);
    visited_list.push_back(start_idx);
    node_map.insert({start_idx,start_node});

    //Astar main loop, until find the goal station
    while(open_list.size()>0){
        close_list.push_back(open_list.top());
        //if the top open node is the goal, return the path
        if(open_list.top()->station_info.name == network[goal_idx].name){
            // cout << "Find the shortest path!" << endl;
            print_path(open_list.top());
            return 0; }

        open_list.pop();
        for(int idx=0;idx < close_list.size();idx++){
        //go over all the closed node
            for(int jdx=0; jdx<network.size();jdx++){
            //expand to serach the neighbors
                double subdistance;
                subdistance = cal_distance(close_list[idx]->station_info, network[jdx]);
                if(subdistance < 320){
                    if(std::find(visited_list.begin(), visited_list.end(), jdx) == visited_list.end()){
                        //append new children node
                        children_node = new Node;
                        g = subdistance + close_list[idx]->g + CP/(close_list[idx]->station_info.rate);
                        h = get_heuristic(network[jdx],network[goal_idx]);
                        children_node->g = g;
                        children_node->f = h+g;
                        children_node->parent = close_list[idx];
                        children_node->station_info = network[jdx];
        
                        visited_list.push_back(jdx);
                        open_list.push(children_node);
                        node_map.insert({jdx,children_node});
                    }
                    else{
                        //update the children node
                        double old_g = node_map[jdx]->g;
                        double new_g = subdistance + close_list[idx]->g + CP/(close_list[idx]->station_info.rate);
                        if(new_g < old_g){
                            g = min(old_g,new_g);
                            h = get_heuristic(network[jdx],network[goal_idx]);
                            node_map[jdx]->g = g;
                            node_map[jdx]->f = g + h;
                            node_map[jdx]->parent = close_list[idx];
                        }
                    }
                }
            }
        }
        close_list.pop_back();
    }
    return 0;
};