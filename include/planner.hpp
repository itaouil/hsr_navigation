#ifndef PLANNER_HPP_
#define PLANNER_HPP_

class Planner
{
public:
    // Contructor (explicit)
    explicit Planner();

    // Destructor (virtual)
    virtual ~Planner();

// Methods
private:
    void dwaPlanning();
    void aStarPlanning();
    void loadOccupancyMap();

// Members
private:
    //TODO: create members 
};

#endif // PLANNER_HPP_
