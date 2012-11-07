#ifndef __MAPPER_HPP__
#define __MAPPER_HPP__

class OccupencyGrid {
   public:
      OccupencyGrid();
      OccupencyGrid(float map_x1,float map_x2,float map_y1,float map_y2,float x_res,float y_res);

      float ** Map;
      
      int M() { return m; }
      int N() { return n; }

      float itoy (const int i) ;
      float jtox (const int j) ;

      void saveMap(string fileName) ;
      void loadMap(string fileName) ;
      
      void fillMap(float x1, float y1, float x2, float y2) ;

      void updateCell (float p,int i,int j) ;

      ~OccupencyGrid();
   private:
      int m,n;
      
      float x1,x2,y1,y1;
      float xRes,yRes;
};

class LaserScanner {
   public:
      LaserScanner();
      LaserScanner(OccupencyGrid * );

      float RMax ;
      float RMin ;
      float AngMax ;
      float AngMin ;
      float AngRes ;

      const float Beta ;  // degrees
      const float Alpha ;   // m
      std::vector<float> ranges;

      OccupencyGrid * grid;

      void callback (const sensor_msgs::LaserScan::ConstPtr& msg);

      ~LaserScanner();
      private:
         void updateMap() ;
};

#endif
