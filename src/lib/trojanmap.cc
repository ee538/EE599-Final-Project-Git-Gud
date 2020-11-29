#include "trojanmap.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

#include <algorithm>
#include <fstream>
#include <locale>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <utility>
#include <unordered_set>
#include <algorithm>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"

//-----------------------------------------------------
// TODO (Students): You do not and should not change the following functions:
//-----------------------------------------------------

/**
 * PrintMenu: Create the menu
 * 
 */
void TrojanMap::PrintMenu() {

  std::string menu =
      "**************************************************************\n"
      "* Select the function you want to execute.                    \n"
      "* 1. Autocomplete - Start                                     \n"
      "* 2. Autocomplete - Any                                       \n"
      "* 3. Find the position                                        \n"
      "* 4. CalculateShortestPath                                    \n"
      "* 5. Travelling salesman problem                              \n"
      "* 6. Traveling salesman problem using 2-opt                   \n"
      "* 7. Traveling salesman problem using 3-opt                   \n"
      "* 8. Exit                                                     \n"
      "**************************************************************\n";
  std::cout << menu << std::endl;
  std::string input;
  getline(std::cin, input);
  char number = input[0];
  switch (number)
  {
  case '1':
  {
    menu =
        "**************************************************************\n"
        "* 1. Autocomplete - Start                                     \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = Autocomplete(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '2':
  {
    menu =
        "**************************************************************\n"
        "* 2. Autocomplete - Any                                       \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a partial location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = Autocomplete_any(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '3':
  {
    menu =
        "**************************************************************\n"
        "* 3. Find the position                                        \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input a location:";
    std::cout << menu;
    getline(std::cin, input);
    auto results = GetPosition(input);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.first != -1) {
      std::cout << "Latitude: " << results.first
                << " Longitude: " << results.second << std::endl;
      PlotPoint(results.first, results.second);
    } else {
      std::cout << "No matched locations." << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '4':
  {
    menu =
        "**************************************************************\n"
        "* 4. CalculateShortestPath                                            "
        "      \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "Please input the start location:";
    std::cout << menu;
    std::string input1;
    getline(std::cin, input1);
    menu = "Please input the destination:";
    std::cout << menu;
    std::string input2;
    getline(std::cin, input2);
    auto results = CalculateShortestPath(input1, input2);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    if (results.size() != 0) {
      for (auto x : results) std::cout << x << std::endl;
      PlotPath(results);
    } else {
      std::cout << "No route from the start point to the destination."
                << std::endl;
    }
    menu = "**************************************************************\n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '5':
  {
    menu =
        "**************************************************************\n"
        "* 5. Travelling salesman problem                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data) {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    PlotPoints(locations);
    std::cout << "Calculating ..." << std::endl;
    auto results = TravellingTrojan(locations);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    CreateAnimation(results.second);
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
           "You could find your animation at src/lib/output.avi.          \n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '6':
  {
    menu =
        "**************************************************************\n"
        "* 6. Travelling salesman problem using 2-opt                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data) {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    PlotPoints(locations);
    std::cout << "Calculating ..." << std::endl;
    auto results = TravellingTrojan_2opt(locations);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    CreateAnimation(results.second);
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
            "You could find your animation at src/lib/output.avi.          \n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '7':
  {
    menu =
        "**************************************************************\n"
        "* 7. Travelling salesman problem using 3-opt                              \n"
        "**************************************************************\n";
    std::cout << menu << std::endl;
    menu = "In this task, we will select N random points on the map and you need to find the path to travel these points and back to the start point.";
    std::cout << menu << std::endl << std::endl;
    menu = "Please input the number of the places:";
    std::cout << menu;
    getline(std::cin, input);
    int num = std::stoi(input);
    std::vector<std::string> keys;
    for (auto x : data) {
      keys.push_back(x.first);
    }
    std::vector<std::string> locations;
    srand(time(NULL));
    for (int i = 0; i < num; i++)
      locations.push_back(keys[rand() % keys.size()]);
    PlotPoints(locations);
    std::cout << "Calculating ..." << std::endl;
    auto results = TravellingTrojan_3opt(locations);
    menu = "*************************Results******************************\n";
    std::cout << menu;
    CreateAnimation(results.second);
    if (results.second.size() != 0) {
      for (auto x : results.second[results.second.size()-1]) std::cout << x << std::endl;
      menu = "**************************************************************\n";
      std::cout << menu;
      std::cout << "The distance of the path is:" << results.first << std::endl;
      PlotPath(results.second[results.second.size()-1]);
    } else {
      std::cout << "The size of the path is 0" << std::endl;
    }
    menu = "**************************************************************\n"
            "You could find your animation at src/lib/output.avi.          \n";
    std::cout << menu << std::endl;
    PrintMenu();
    break;
  }
  case '8':
          break;
  default:
    std::cout << "Please select 1 - 8." << std::endl;
    PrintMenu();
    break;
  }
}


/**
 * CreateGraphFromCSVFile: Read the map data from the csv file
 * 
 */
void TrojanMap::CreateGraphFromCSVFile() {
  std::fstream fin;
  fin.open("src/lib/map.csv", std::ios::in);
  std::string line, word;

  getline(fin, line);
  while (getline(fin, line)) {
    std::stringstream s(line);

    Node n;
    int count = 0;
    while (getline(s, word, ',')) {
      word.erase(std::remove(word.begin(), word.end(), '\''), word.end());
      word.erase(std::remove(word.begin(), word.end(), '"'), word.end());
      word.erase(std::remove(word.begin(), word.end(), '['), word.end());
      word.erase(std::remove(word.begin(), word.end(), ']'), word.end());
      if (count == 0)
        n.id = word;
      else if (count == 1)
        n.lat = stod(word);
      else if (count == 2)
        n.lon = stod(word);
      else if (count == 3)
        n.name = word;
      else {
        word.erase(std::remove(word.begin(), word.end(), ' '), word.end());
        n.neighbors.push_back(word);
      }
      count++;
    }
    data[n.id] = n;
  }
  fin.close();
}

/**
 * PlotPoint: Given a location id, plot the point on the map
 * 
 * @param  {std::string} id : location id
 */
void TrojanMap::PlotPoint(std::string id) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(data[id].lat, data[id].lon);
  cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}
/**
 * PlotPoint: Given a lat and a lon, plot the point on the map
 * 
 * @param  {double} lat : latitude
 * @param  {double} lon : longitude
 */
void TrojanMap::PlotPoint(double lat, double lon) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto result = GetPlotLocation(lat, lon);
  cv::circle(img, cv::Point(int(result.first), int(result.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPath: Given a vector of location ids draws the path (connects the points)
 * 
 * @param  {std::vector<std::string>} location_ids : path
 */
void TrojanMap::PlotPath(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
  cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
             cv::Scalar(0, 0, 255), cv::FILLED);
  for (auto i = 1; i < location_ids.size(); i++) {
    auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
    auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
    cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
    cv::line(img, cv::Point(int(start.first), int(start.second)),
             cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
             LINE_WIDTH);
  }
  cv::startWindowThread();
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}

/**
 * PlotPoints: Given a vector of location ids draws the points on the map (no path).
 * 
 * @param  {std::vector<std::string>} location_ids : points
 */
void TrojanMap::PlotPoints(std::vector<std::string> &location_ids) {
  std::string image_path = cv::samples::findFile("src/lib/input.jpg");
  cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
  for (auto x : location_ids) {
    auto result = GetPlotLocation(data[x].lat, data[x].lon);
    cv::circle(img, cv::Point(result.first, result.second), DOT_SIZE,
               cv::Scalar(0, 0, 255), cv::FILLED);
  }
  cv::imshow("TrojanMap", img);
  cv::waitKey(1);
}


/**
 * CreateAnimation: Create the videos of the progress to get the path
 * 
 * @param  {std::vector<std::vector<std::string>>} path_progress : the progress to get the path
 */
void TrojanMap::CreateAnimation(std::vector<std::vector<std::string>> path_progress){
  cv::VideoWriter video("src/lib/output.avi", cv::VideoWriter::fourcc('M','J','P','G'), 10, cv::Size(1248,992));
  for(auto location_ids: path_progress) {
    std::string image_path = cv::samples::findFile("src/lib/input.jpg");
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    auto start = GetPlotLocation(data[location_ids[0]].lat, data[location_ids[0]].lon);
    cv::circle(img, cv::Point(int(start.first), int(start.second)), DOT_SIZE,
              cv::Scalar(0, 0, 255), cv::FILLED);
    for (auto i = 1; i < location_ids.size(); i++) {
      auto start = GetPlotLocation(data[location_ids[i - 1]].lat, data[location_ids[i - 1]].lon);
      auto end = GetPlotLocation(data[location_ids[i]].lat, data[location_ids[i]].lon);
      cv::circle(img, cv::Point(int(end.first), int(end.second)), DOT_SIZE,
                cv::Scalar(0, 0, 255), cv::FILLED);
      cv::line(img, cv::Point(int(start.first), int(start.second)),
              cv::Point(int(end.first), int(end.second)), cv::Scalar(0, 255, 0),
              LINE_WIDTH);
    }
    video.write(img);
    cv::imshow("TrojanMap", img);
    cv::waitKey(1);
  }
	video.release();
}
/**
 * GetPlotLocation: Transform the location to the position on the map
 * 
 * @param  {double} lat         : latitude 
 * @param  {double} lon         : longitude
 * @return {std::pair<double, double>}  : position on the map
 */
std::pair<double, double> TrojanMap::GetPlotLocation(double lat, double lon) {
  std::pair<double, double> bottomLeft(34.01001, -118.30000);
  std::pair<double, double> upperRight(34.03302, -118.26502);
  double h = upperRight.first - bottomLeft.first;
  double w = upperRight.second - bottomLeft.second;
  std::pair<double, double> result((lon - bottomLeft.second) / w * 1248,
                                   (1 - (lat - bottomLeft.first) / h) * 992);
  return result;
}

//-----------------------------------------------------
// TODO: Student should implement the following:
//-----------------------------------------------------
/**
 * GetLat: Get the latitude of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {double}         : latitude
 */
double TrojanMap::GetLat(std::string id) 
{
  if(data.find(id)==data.end())
    return -1;
  return data[id].lat;
}

/**
 * GetLon: Get the longitude of a Node given its id. 
 * 
 * @param  {std::string} id : location id
 * @return {double}         : longitude
 */
double TrojanMap::GetLon(std::string id) 
{ 
  if(data.find(id)==data.end())
    return -1;
  return data[id].lon; 
}

/**
 * GetName: Get the name of a Node given its id.
 * 
 * @param  {std::string} id : location id
 * @return {std::string}    : name
 */
std::string TrojanMap::GetName(std::string id) 
{ 
  if(data.find(id)==data.end())
    return "";
  return data[id].name; 
}

/**
 * GetNeighborIDs: Get the neighbor ids of a Node.
 * 
 * @param  {std::string} id            : location id
 * @return {std::vector<std::string>}  : neighbor ids
 */
std::vector<std::string> TrojanMap::GetNeighborIDs(std::string id) {
    std::vector<std::string> result;
    if(data.find(id)==data.end())
      return result;
    result = data[id].neighbors;
    return result;
}


/**
 * CalculateDistance: Get the distance between 2 nodes. 
 * 
 * @param  {Node} a  : node a
 * @param  {Node} b  : node b
 * @return {double}  : distance in mile
 */
double TrojanMap::CalculateDistance(const Node &a, const Node &b) {
  // TODO: Use Haversine Formula:
  // dlon = lon2 - lon1;
  // dlat = lat2 - lat1;
  // a = (sin(dlat / 2)) ^ 2 + cos(lat1) * cos(lat2) * (sin(dlon / 2)) ^ 2;
  // c = 2 * arcsin(min(1, sqrt(a)));
  // distances = 3961 * c;

  // where 3961 is the approximate radius of the earth at the latitude of
  // Washington, D.C., in miles
  double lat1 = a.lat, lat2 = b.lat, lon1 = a.lon, lon2 = b.lon;
  double dLat = (lat2 - lat1) * M_PI / 180.0; 
  double dLon = (lon2 - lon1) * M_PI / 180.0; 

  lat1 = (lat1) * M_PI / 180.0; 
  lat2 = (lat2) * M_PI / 180.0; 

  double at = pow(sin(dLat / 2), 2) + pow(sin(dLon / 2), 2) * cos(lat1) * cos(lat2); 
  double rad = 6371; 
  double c = 2 * asin(sqrt(at)); 
  return rad * c; 
}

/**
 * CalculatePathLength: Calculates the total path length for the locations inside the vector.
 * 
 * @param  {std::vector<std::string>} path : path
 * @return {double}                        : path length
 */
double TrojanMap::CalculatePathLength(const std::vector<std::string> &path) 
{
  double sum = 0;
  for(auto temp = path.begin();temp!=path.end()-1;temp++)
  {
    sum += CalculateDistance(data[*temp],data[*(temp+1)]);
  }  
  return sum;
}

/**
 * Autocomplete: Given a parital name return all the possible locations with
 * partial name as the prefix.
 *
 * @param  {std::string} name          : partial name
 * @return {std::vector<std::string>}  : a vector of full names
 */
std::vector<std::string> TrojanMap::Autocomplete(std::string name) {
  std::vector<std::string> results;
  for(auto temp : data)
  {
    std::string x(temp.second.name.length(),'a');
    std::transform(temp.second.name.begin(), temp.second.name.end(), x.begin(),[](unsigned char c){ return std::tolower(c); });
    std::transform(name.begin(), name.end(), name.begin(),[](unsigned char c){ return std::tolower(c); });
    std::size_t found = x.find(name);
    if(found == std::string::npos || found!=0)
      continue;
    results.push_back(temp.second.name);
  }
  return results;
}

std::vector<std::string> TrojanMap::Autocomplete_any(std::string name) {
  std::vector<std::string> results;
  for(auto temp : data)
  {
    std::string x(temp.second.name.length(),'a');
    std::transform(temp.second.name.begin(), temp.second.name.end(), x.begin(),[](unsigned char c){ return std::tolower(c); });
    std::transform(name.begin(), name.end(), name.begin(),[](unsigned char c){ return std::tolower(c); });
    std::size_t found = x.find(name);
    if(found == std::string::npos)
      continue;
    results.push_back(temp.second.name);
  }
  return results;
}

/**
 * GetPosition: Given a location name, return the position.
 *
 * @param  {std::string} name          : location name
 * @return {std::pair<double,double>}  : (lat, lon)
 */
std::pair<double, double> TrojanMap::GetPosition(std::string name) {
  std::pair<double, double> results(-1, -1);
  for(auto temp : data)
  {
    if(temp.second.name.compare(name) ==0)
    {
      results.first = temp.second.lat;
      results.second = temp.second.lon;
    }
    
  }
  return results;
}

/**
 * CalculateShortestPath: Given 2 locations, return the shortest path which is a
 * list of id.
 *
 * @param  {std::string} location1_name     : start
 * @param  {std::string} location2_name     : goal
 * @return {std::vector<std::string>}       : path
 */

std::string mindist(std::map<std::string, double> dist, std::map<std::string,bool> visited)
{
  double min = DBL_MAX;
  std::string min_id;
  for(auto temp:dist)
  {
    if((visited[temp.first] == false) && (dist[temp.first] <= min))
    {
      min = dist[temp.first];
      min_id = temp.first;
    }
  }
  if(min_id.compare("") == 0)
    std::cout<<"Error!"<<std::endl;
  return min_id;
}

void pathfind(std::map<std::string, std::string> parent, std::string id, std::vector<std::string> &x)
{
  if(parent[id].compare("-1")==0)
    return;
  pathfind(parent, parent[id],x);
  x.push_back(id);
}

std::vector<std::string> TrojanMap::CalculateShortestPath(std::string location1_name, std::string location2_name) 
{
  std::vector<std::string> x;
  std::map<std::string, double> dist;
  std::map<std::string, bool> visited;
  std::map<std::string, std::string> parent;
  std::string src, dst;
  for(auto temp : data)
  {
    dist[temp.first] = DBL_MAX;
    parent[temp.first] = "-1";
    visited[temp.first] = false;
    if(temp.second.name.compare(location1_name)==0)
      src = temp.first;
    if(temp.second.name.compare(location2_name)==0)
      dst = temp.first;
  }
  dist[src] = 0;
  for(int i=0; i<dist.size()-1; i++)
  {
    std::string u = mindist(dist,visited);
    visited[u] = true;
    for(auto v : dist)
    {
      std::vector<std::string> temp = GetNeighborIDs(u);
      if(visited[v.first] == false && std::find(temp.begin(), temp.end(), v.first) !=temp.end() && dist[u] + CalculateDistance(data[u],data[v.first]) < dist[v.first])
      {
        parent[v.first] = u;
        dist[v.first] = dist[u] + CalculateDistance(data[u],data[v.first]);
      }
    }
  }
  x.push_back(src);
  pathfind(parent, dst, x);
  
  return x;
}

/**
 * Travelling salesman problem: Given a list of locations, return the shortest
 * path which visit all the places and back to the start point.
 *
 * @param  {std::vector<std::string>} input : a list of locations needs to visit
 * @return {std::pair<double, std::vector<std::vector<std::string>>} : a pair of total distance and the all the progress to get final path
 */
std::pair<double, std::vector<std::vector<std::string>>> TrojanMap::TravellingTrojan(std::vector<std::string> &location_ids) 
{
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::string> vertex = location_ids;
  vertex.erase(vertex.begin());
  std::sort(vertex.begin(), vertex.end());
  std::vector<std::vector<std::string>> paths;
  std::vector<std::string> currpath;
  double min_path = DBL_MAX;
  do
  {
    double curr_cost = 0;
    std::string k = location_ids.front();

    for(auto temp: vertex)
    {
      curr_cost += CalculateDistance(data[k],data[temp]);
      k = temp;
    } 

    curr_cost += CalculateDistance(data[k],data[location_ids.front()]);
    if(min_path>curr_cost)
    {
      currpath.clear();
      currpath.push_back(location_ids[0]);
      for(auto x: vertex)
        currpath.push_back(x);
      currpath.push_back(location_ids[0]);
      paths.push_back(currpath);
    }
    min_path = std::min(min_path, curr_cost);
  }while(next_permutation(vertex.begin(),vertex.end()));
  results.first = min_path;
  results.second = paths;
  return results;
} 

std::pair<double, std::vector<std::vector<cv::String>>> TrojanMap::TravellingTrojan_2opt(std::vector<cv::String> &location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::string> vertex = location_ids;
  vertex.push_back(location_ids.front());
  int length = vertex.size();
  int improve = 0;
  while(improve<20)
  {
    start_again: double best_dist = CalculatePathLength(vertex);
    for(int i=1; i<length-2; i++)
    {
      for(int j=i+1; j<length-1; j++)
      {
        std::vector<std::string> temp = vertex;
        std::reverse(temp.begin()+i,temp.begin()+j+1);
        double new_dist = CalculatePathLength(temp);
        if(new_dist < best_dist)
        {
          improve = 0;
          vertex.clear();
          vertex = temp;
          best_dist = new_dist;
          results.second.push_back(temp);
          goto start_again;
        }
      }
    }
    improve++;
  }
  results.first = CalculatePathLength(vertex);
  return results;
}

std::pair<double, std::vector<std::vector<cv::String>>> TrojanMap::TravellingTrojan_3opt(std::vector<cv::String> &location_ids)
{
  std::pair<double, std::vector<std::vector<std::string>>> results;
  std::vector<std::string> vertex = location_ids;
  vertex.push_back(location_ids.front());
  int length = vertex.size();
  bool improve = false;
  std::vector<std::string> temp = vertex;
  while(!improve)
  {
    improve=false;
    for(int counter1=0; counter1<length; counter1++)
    {
      int i = counter1;
      for(int counter2=1; counter2<length-2; counter2++)
      {
        int j = (i+counter2)%length;
        for(int counter3=counter2+1; counter3<length; counter3++)
        {
          int k = (i+counter3)%length;
          int x1 = i, x2 = (i+1)%length, y1 = j, y2 = (j+1)%length, z1 = k, z2 = (k+1)%length;

          double d1 = CalculateDistance(data[temp[x1]],data[temp[x2]]) + CalculateDistance(data[temp[z1]],data[temp[z2]]) - CalculateDistance(data[temp[x1]],data[temp[z1]]) - CalculateDistance(data[temp[x2]],data[temp[z2]]);
          double d2 = CalculateDistance(data[temp[y1]],data[temp[y2]]) + CalculateDistance(data[temp[z1]],data[temp[z2]]) - CalculateDistance(data[temp[y1]],data[temp[z1]]) - CalculateDistance(data[temp[y2]],data[temp[z2]]);
          double d3 = CalculateDistance(data[temp[x1]],data[temp[x2]]) + CalculateDistance(data[temp[y1]],data[temp[y2]]) - CalculateDistance(data[temp[y1]],data[temp[z1]]) - CalculateDistance(data[temp[y2]],data[temp[z2]]);
          double d4 = CalculateDistance(data[temp[x1]],data[temp[x2]]) + CalculateDistance(data[temp[y1]],data[temp[y2]]) + CalculateDistance(data[temp[z1]],data[temp[z2]]) - CalculateDistance(data[temp[x1]],data[temp[y1]]) - CalculateDistance(data[temp[x2]],data[temp[z1]]) - CalculateDistance(data[temp[y2]],data[temp[z2]]);
          double d5 = CalculateDistance(data[temp[x1]],data[temp[x2]]) + CalculateDistance(data[temp[y1]],data[temp[y2]]) + CalculateDistance(data[temp[z1]],data[temp[z2]]) - CalculateDistance(data[temp[x1]],data[temp[z1]]) - CalculateDistance(data[temp[y2]],data[temp[x2]]) - CalculateDistance(data[temp[y1]],data[temp[z2]]);
          double d6 = CalculateDistance(data[temp[x1]],data[temp[x2]]) + CalculateDistance(data[temp[y1]],data[temp[y2]]) + CalculateDistance(data[temp[z1]],data[temp[z2]]) - CalculateDistance(data[temp[x1]],data[temp[y2]]) - CalculateDistance(data[temp[z1]],data[temp[y1]]) - CalculateDistance(data[temp[x2]],data[temp[z2]]);
          double d7 = CalculateDistance(data[temp[x1]],data[temp[x2]]) + CalculateDistance(data[temp[y1]],data[temp[y2]]) + CalculateDistance(data[temp[z1]],data[temp[z2]]) - CalculateDistance(data[temp[x1]],data[temp[y2]]) - CalculateDistance(data[temp[z1]],data[temp[x2]]) - CalculateDistance(data[temp[y1]],data[temp[z2]]);
          
          if(d1>0)
          {
            std::reverse(temp.begin()+((k+1)%length),temp.begin()+j);
            improve = true;
            results.second.push_back(temp);
            continue;
          }
          if(d2>0)
          {
            std::reverse(temp.begin()+((j+1)%length),temp.begin()+k);
            improve = true;
            results.second.push_back(temp);
            continue;
          }
          if(d3>0)
          {
            std::reverse(temp.begin()+((i+1)%length),temp.begin()+j);
            improve = true;
            results.second.push_back(temp);
            continue;
          }
          if(d4>0)
          {
            std::reverse(temp.begin()+((j+1)%length),temp.begin()+k);
            std::reverse(temp.begin()+((i+1)%length),temp.begin()+j);
            improve = true;
            results.second.push_back(temp);
            continue;
          }
          if(d5>0)
          {
            std::reverse(temp.begin()+((k+1)%length),temp.begin()+i);
            std::reverse(temp.begin()+((i+1)%length),temp.begin()+j);
            results.second.push_back(temp);
            improve = true;
            continue;
          }
          if(d6>0)
          {
            std::reverse(temp.begin()+((k+1)%length),temp.begin()+i);
            std::reverse(temp.begin()+((j+1)%length),temp.begin()+k);
            results.second.push_back(temp);
            improve = true;
            continue;
          }
          if(d7>0)
          {
            std::reverse(temp.begin()+((k+1)%length),temp.begin()+i);
            std::reverse(temp.begin()+((i+1)%length),temp.begin()+j);
            std::reverse(temp.begin()+((j+1)%length),temp.begin()+k);
            results.second.push_back(temp);
            improve = true;
            continue;
          }
        }
      }
    }
  }
  // std::cout<<"Done";
  results.first = CalculatePathLength(results.second[results.second.size()-1]);
  for(auto x : results.second[results.second.size()-1])
    std::cout<<x<<" ";
  std::cout<<std::endl;
  return results;
}  
