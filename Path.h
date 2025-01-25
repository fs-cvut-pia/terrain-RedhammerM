#ifndef PATH_P
#define PATH_P

#include "TerrainMap.h"

// Abstract class which needs to be extended to contain the actual path finding algorithm

class Path {
public:
    Path(TerrainMap& m, std::string name_in, Point start_in, Point finish_in);
    virtual bool find() = 0;     // Implement this method to find the route and save it in vector<Point> path
    void printStats() const;     // Print out path statistics
    void saveToFile() const;     // Save path to file "name.dat"
    std::string getName() const; // Returns path name
protected:
    TerrainMap& map;
    std::vector<Point> path;
    const Point start; 
    const Point ciel;
private:
    std::string name;
};

//triedy pre jednotlive typy cesty:
class lietadlo : public Path {
public:
    lietadlo(TerrainMap& m, Point start_in, Point finish_in) : Path(m, "lietadlo", start_in, finish_in) {}
    bool find();
};

class lod : public Path {
public:
    lod(TerrainMap& m, Point start_in, Point finish_in) : Path(m, "lod", start_in, finish_in) {}
    bool find();
};

class cesta : public Path {
public:
    cesta(TerrainMap& m, Point start_in, Point finish_in) : Path(m, "cesta", start_in, finish_in) {}
    bool find();
};

class trajekt : public Path {
public:
    trajekt(TerrainMap& m, Point start_in, Point finish_in) : Path(m, "trajekt", start_in, finish_in) {}
    bool find();
};

class vlak : public Path {
public:
    vlak(TerrainMap& m, Point start_in, Point finish_in) : Path(m, "vlak", start_in, finish_in) {}
    bool find();
};

#endif