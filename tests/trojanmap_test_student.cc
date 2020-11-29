#include "src/lib/trojanmap.h"

#include <map>
#include <vector>

#include "gtest/gtest.h"


//5237417650,34.0257016,-118.2843512,Target,['6813379479']
//5237417651,34.025187,-118.2841713,Bank of America,"['5261316289', '6047234450']"
//5680945528,34.0271548,-118.2735783,,"['4835549598', '123292076', '5514004014']"
//5695236164,34.0282131,-118.2756114,Popeyes Louisiana Kitchen,['4835551070']


TEST(TrojanMapTest, GetLat_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Get the Latitude of a Node given its id.
  double out = m.GetLat("5237417650");
  double out_expected =  34.0257016;
  EXPECT_EQ(out, out_expected);

  double out1 = m.GetLat("5237417651");
  double out_expected1 =  34.025187;
  EXPECT_EQ(out1, out_expected1);
}

TEST(TrojanMapTest, GetLon_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Get the Longitude of a Node given its id.
  double out = m.GetLon("5237417650");
  double out_expected =  -118.2843512;
  EXPECT_EQ(out, out_expected);

  double out1 = m.GetLon("5237417651");
  double out_expected1 =  -118.2841713;
  EXPECT_EQ(out1, out_expected1);
}


TEST(TrojanMapTest, GetName_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Get the name of a Node given its id.
  std::string out = m.GetName("5237417650");
  std::string out_expected =  "Target";
  EXPECT_EQ(out, out_expected);

  std::string out1 = m.GetName("5237417651");
  std::string out_expected1 =  "Bank of America";
  EXPECT_EQ(out1, out_expected1);

  std::string out2 = m.GetName("5680945528");
  std::string out_expected2 =  "";
  EXPECT_EQ(out1, out_expected1);
}

TEST(TrojanMapTest, GetNeighborIDs_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Get the neighbor ids of a Node.
   std::vector<std::string> out = m.GetNeighborIDs("5237417650");
   std::vector<std::string> out_expected =  {"6813379479"};
  EXPECT_EQ(out, out_expected);

   std::vector<std::string> out1 = m.GetNeighborIDs("5237417651");
   std::vector<std::string> out_expected1 =  {"5261316289", "6047234450"};
  EXPECT_EQ(out1, out_expected1);

   std::vector<std::string> out2 = m.GetNeighborIDs("5680945528");
   std::vector<std::string> out_expected2 =  {"4835549598", "123292076", "5514004014"};
  EXPECT_EQ(out2, out_expected2);
}

TEST(TrojanMapTest, CalculateDistance_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
 // Get the distance between 2 nodes
  Node a,b;
  a.lat = 34.0257016;
  a.lon = -118.2843512;
  b.lat = 34.025187;
  b.lon = -118.2841713;
  double out = m.CalculateDistance(a,b) ;
  double out_expected =  0.05957430704331041077;
  EXPECT_EQ(out, out_expected);
}

TEST(TrojanMapTest, CalculatePathLength_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  // Calculates the total path length for the locations inside the vector.
  double out = m.CalculatePathLength({"4732965439","1932939400","7249047942","123292049","122470423","4399914023","4399914025","1837202707","1837202710","1837202714","4399914037","4399914040","1837202706","123292047","6939732877","123292045","123292100","1378231753","2193435032","1841835270","4011837222","1841835282","122855868","63785495","5680945549","2613157014","4835551230","7735888669","6813565292","122719205","6813565294","4835551232","4835551104","4012842272","4835551103","123178841","6813565313","122814435","6813565312","4835551101","4835551096","3088547686"}) ;
  double out_expected =  1.260621036699601794;
  EXPECT_EQ(out, out_expected);
}


TEST(TrojanMapTest, Autocomplete1_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto names = m.Autocomplete("T");
  std::vector<std::string> gt1 = {"Traveler39s Fountain", "Tommy Trojan", "Trinity Elementary School", "The Row House", "Traveler", "Tutor Campus Center Piano", "Trader Joe39s", "Target", "The Pearl", "The Caribbean Apartments", "Trejo39s Taco", "Tap Two Blue", "Trojan Grounds Starbucks", "The Mirage", "Troy View Swimming Pool"};
  EXPECT_EQ(names, gt1);
  names = m.Autocomplete("Tr");
  std::vector<std::string> gt2 = {"Traveler39s Fountain", "Trinity Elementary School", "Traveler", "Trader Joe39s", "Trejo39s Taco", "Trojan Grounds Starbucks", "Troy View Swimming Pool"};
  EXPECT_EQ(names, gt2);
  names = m.Autocomplete("tro");
  std::vector<std::string> gt3 = {"Trojan Grounds Starbucks", "Troy View Swimming Pool"};
  EXPECT_EQ(names, gt3);
  names = m.Autocomplete("1234asdff");
  std::vector<std::string> gt4 = {};
  EXPECT_EQ(names, gt4);
}

TEST(TrojanMapTest, Autocomplete1_any) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto names = m.Autocomplete_any("roj");
  std::vector<std::string> gt1 = {"Tommy Trojan", "Trojan Grounds Starbucks"};
  EXPECT_EQ(names, gt1);
  auto names1 = m.Autocomplete_any("my");
  std::vector<std::string> gt2 = {"Tommy Trojan"};
  EXPECT_EQ(names1, gt2);
  auto names3 = m.Autocomplete_any("Ral");
  std::vector<std::string> gt3 = {"Ralphs", "St John39s CathedralHope Net", "Historic SouthCentral"};
}



TEST(TrojanMapTest, FindPosition1_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto position = m.GetPosition("Saint James Park");
  std::pair<double, double> gt1(34.0311237,-118.2800746);
  EXPECT_EQ(position, gt1);
  position = m.GetPosition("John Adams Middle School");
  std::pair<double, double> gt2(34.02286,-118.272);
  EXPECT_EQ(position, gt2);
  position = m.GetPosition("Warning Skate Shop");
  std::pair<double, double> gt3(34.0303996,-118.2913965);
  EXPECT_EQ(position, gt3);
  position = m.GetPosition("Impossible");
  std::pair<double, double> gt4(-1,-1);
  EXPECT_EQ(position, gt4);
}

TEST(TrojanMapTest, GetPosition2_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto position = m.GetPosition("Anna39s Store");
  std::pair<double, double> gt1(34.0290611,-118.2916977);
  EXPECT_EQ(position, gt1);
  position = m.GetPosition("University Park");
  std::pair<double, double> gt2(34.027449,-118.2839493);
  EXPECT_EQ(position, gt2);
  position = m.GetPosition("ExpoVermont");
  std::pair<double, double> gt3(34.01825,-118.2917403);
  EXPECT_EQ(position, gt3);
}

TEST(TrojanMapTest, CalculateShortestPath1_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath("Department of Motor Vehicles", "Kentucky Fried Chicken");
  std::vector<std::string> gt{"4732965439","1932939400","7249047942","123292049","122470423","4399914023","4399914025","1837202707","1837202710","1837202714","4399914037","4399914040","1837202706","123292047","6939732877","123292045","123292100","1378231753","2193435032","1841835270","4011837222","1841835282","122855868","63785495","5680945549","2613157014","4835551230","7735888669","6813565292","122719205","6813565294","4835551232","4835551104","4012842272","4835551103","123178841","6813565313","122814435","6813565312","4835551101","4835551096","3088547686"};
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  path = m.CalculateShortestPath("Kentucky Fried Chicken", "Department of Motor Vehicles");
  std::reverse(gt.begin(),gt.end());
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

TEST(TrojanMapTest, CalculateShortestPath2_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath("Martin Luther King Jr Blvd  Coliseum", "Subway");
  std::vector<std::string> gt{
      "269633667","6815813004","348123159","348123160","4020099351","348123342","4020099346","348123344","1870795193","122609808","4020099340","348123012","1870797772","5617976418","21302801","1870795205","1870797882","1855150081","1759017530"};
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  path = m.CalculateShortestPath("Subway", "Martin Luther King Jr Blvd  Coliseum");
  std::reverse(gt.begin(),gt.end());
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
}

TEST(TrojanMapTest, CalculateShortestPath3_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  auto path = m.CalculateShortestPath("ChickfilA", "Historic SouthCentral");
  std::vector<std::string> gt{
      "4547476733","6820935911","1837212101","1837212103","6813411589","216155217","6813411590","1472141024","6813405280","348121864","6813405282","348122195","4399914021","348122196","348122197","348122198","4015405538","6820972454","269633045","269633572","1377766078","5541778221","6820972473","269633046","269633015","269633016","7249047931","269633017","269633018","269633019","6820972484","269633021","6820972480","6820982899","21302782","6820982898","4012792178","1732243772","1732243631","6807583648","122619498","4343588868","1716288017","4343588869","441893332","123015386","250768851","1758031815","123153849","250768852","1758031816","123732661","123380341","123380343","123009684","67666164","7360424710"};
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
  
  path = m.CalculateShortestPath("Historic SouthCentral", "ChickfilA");
  std::reverse(gt.begin(),gt.end());
  std::cout << "My path length: "  << m.CalculatePathLength(path) << "miles" << std::endl;
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  EXPECT_EQ(path, gt);
} 

TEST(TrojanMapTest, TSP1_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1837212101", "7424270441", "6813405280", "216155217", "4015203110", "6820935911","348123344"};
  auto result = m.TravellingTrojan(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl;
  std::vector<std::string> gt{"1837212101","6820935911","7424270441","348123344","4015203110","6813405280","216155217","1837212101"};
  std::cout << "GT path length: " << m.CalculatePathLength(gt) << "miles" << std::endl;
  bool flag = false;
  if (gt == result.second[result.second.size()-1])
    flag = true;
  std::reverse(gt.begin(),gt.end());
  if (gt == result.second[result.second.size()-1])
    flag = true;
  
  EXPECT_EQ(flag, true);
}

TEST(TrojanMapTest, 2opt1_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1837212101", "7424270441", "6813405280", "216155217", "4015203110", "6820935911","348123344"};
  auto result = m.TravellingTrojan_2opt(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl;
  
  bool flag = false;

  if( m.CalculatePathLength(result.second[result.second.size()-1]) < m.CalculatePathLength(result.second[result.second.size()-2])){
    flag = true;
  }
    

  EXPECT_EQ(flag, true);

}

TEST(TrojanMapTest, 3opt1_) {
  TrojanMap m;
  m.CreateGraphFromCSVFile();
  std::vector<std::string> input{"1837212101", "7424270441", "6813405280", "216155217", "4015203110", "6820935911","348123344"};
  auto result = m.TravellingTrojan_3opt(input);
  std::cout << "My path length: " << result.first << "miles" << std::endl;
  
  bool flag = false;

  if( m.CalculatePathLength(result.second[result.second.size()-1]) < m.CalculatePathLength(result.second[0])){
    flag = true;
  }
    

  EXPECT_EQ(flag, true);

}
