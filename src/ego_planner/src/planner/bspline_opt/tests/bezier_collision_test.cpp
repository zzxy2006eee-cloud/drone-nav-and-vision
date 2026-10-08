#include <bspline_opt/bezier_collision.h>
#include <iostream>
#include <fstream>
#include <set>
#include <tuple>
#include <limits>
using namespace ego_planner;
using V=Eigen::Vector3d;
int main(int argc,char**argv) {
 int checks=0;
 auto require=[&](bool v,const char*name){++checks;if(!v){std::cerr<<"FAIL "<<name<<"\n";std::exit(1);}};
 std::set<std::tuple<int,int,int>> cells;
 auto occupied=[&](const Eigen::Vector3i&i){return cells.count(std::make_tuple(i.x(),i.y(),i.z()))>0;};
 auto check=[&](CubicHull h,double margin=0.){std::size_t n=0;double f=0.;return cubicHullFree(h,V(-1,-1,-1),.1,occupied,n,f,margin);};
 CubicHull line{{V(0,0,0),V(.2,0,0),V(.4,0,0),V(.6,0,0)}};
 require(check(line),"clear line");
 cells.emplace(12,10,10);require(!check(line),"continuous interior collision");
 cells.clear();cells.emplace(15,10,10);require(!check(line),"last third collision");
 cells.clear();cells.emplace(11,10,10);
 CubicHull face{{V(.2,0,0),V(.2,0,0),V(.2,0,0),V(.2,0,0)}};
 require(!check(face),"closed boundary contact");
 cells.clear();cells.emplace(8,10,10);
 CubicHull negative{{V(-.15,0,0),V(-.15,0,0),V(-.15,0,0),V(-.15,0,0)}};
 require(!check(negative),"negative coordinates");
 cells.clear();cells.emplace(14,14,10);
 CubicHull arc{{V(0,0,0),V(0,1,0),V(1,1,0),V(1,0,0)}};
 require(check(arc),"intersecting control hull but clear actual curve");
 cells.clear();cells.emplace(12,12,10);
 CubicHull close{{V(0,.151,0),V(.2,.151,0),V(.4,.151,0),V(.6,.151,0)}};
 require(check(close),"unexpanded clear curve");require(!check(close,.05),"five centimeter planning reserve");
 cells.clear();auto invalid=line;invalid[1].x()=std::numeric_limits<double>::quiet_NaN();
 require(!check(invalid),"invalid coordinates");
 std::size_t nodes=10000;double fraction=0.;require(!cubicHullFree(line,V(-1,-1,-1),.1,occupied,nodes,fraction),"bounded nodes");
 nodes=0;require(!cubicHullFree(line,V(-1,-1,-1),0.,occupied,nodes,fraction),"invalid resolution");
 require(!check(line,-.1),"invalid margin");
 if(argc>1) {
  cells.clear();std::ifstream input(std::string(argv[1])+"/occupied.bin",std::ios::binary);
  int32_t i[3];while(input.read(reinterpret_cast<char*>(i),sizeof(i)))cells.emplace(i[0],i[1],i[2]);
  require(!cells.empty(),"recorded occupancy loaded");
  std::ifstream spans(std::string(argv[1])+"/hulls.bin",std::ios::binary);
  double xyz[12];bool safe=true;int count=0;nodes=0;
  while(spans.read(reinterpret_cast<char*>(xyz),sizeof(xyz))) {
   ++count;CubicHull h;for(int j=0;j<4;++j)h[j]=V(xyz[3*j],xyz[3*j+1],xyz[3*j+2]);
   if(!cubicHullFree(h,V(-15,-15,.5),.1,occupied,nodes,fraction))safe=false;
  }
  require(count>0,"recorded full curve loaded");require(!safe,"reject actual unsafe published candidate");
 }
 std::cout<<"PASS "<<checks<<" continuous curve checks\n";
}
