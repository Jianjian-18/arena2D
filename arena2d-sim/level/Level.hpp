/* Author: Cornelius Marx */

#ifndef LEVEL_H
#define LEVEL_H

#include <vector>
#include <arena/PhysicsWorld.hpp>
#include <arena/Robot.hpp>
#include <arena/RectSpawn.hpp>
#include <engine/f_math.h>
#include <engine/GlobalSettings.hpp>
#include <engine/zVector2d.hpp>

#include <arena/ConsoleParameters.hpp>

#define LEVEL_STATIC_FRICTION 		1.0f 		// friction of static obstacles
#define LEVEL_STATIC_RESTITUTION	0.0f 		// restitution ('bouncyness') of static obstacles
#define LEVEL_GOAL_SPAWN_COLOR 		0xF3C90E70 	// color of the area marking possible goal positions


/* initializer for level */
struct LevelDef{
	LevelDef(b2World *w, Robot * b): world(w), robot(b){}

	/* world to create level in */
	b2World * world;

	/* robot that is controlled by the agent */
	Robot * robot;
};

/* base class for all levels */
class Level{
public:
	/* constructor
	 * @param w Box2D world to create level in
	 */
	Level(const LevelDef &);

	/* destructor
	 */
	virtual ~Level();

	/* called after every complete simulation step (not on every step iteration)
	 * use this function to implement any additional level logic (e.g. change velocity of moving obstacles)
	 */
	virtual void update(){}


	/* spawn goal at specified position
	 * @param pos position of the goal
	 */
	virtual void spawnGoal(const b2Vec2 & pos);

	/* reset level, always called before the start of a new episode
	 * @param robot_position_reset if set to false, robot_position is expected to not change for the new level configuration (if possible)
	 */
	virtual void reset(bool robot_position_reset){ if(robot_position_reset){resetRobotToCenter();} randomGoalSpawnUntilValid();}

	/* called for every frame to be drawn on the screen
	 * use this function for additional visualizations
	 */
	virtual void render(){}

	/* render spawn area of goal (possible goal positions)
	 * use command 'show goal_spawn' to show the goal spawn area
	 * the base implementation of this function uses _goalSpawnArea to draw the goal spawn area as rectangles
	 */
	virtual void renderGoalSpawn();

	/* provide the agent with additional data generated by level
	 * @param data any values pushed into this vector will be passed to the agent as additional observations
	 */
	virtual void getAgentData(std::vector<float> & data){}


	/* provide the obstacle information 
	 * @param coordinate data for all robot obstacle
	 */
	virtual void getRobotAgentsData(std::vector<float> & data){}
	/* get level specific reward, called after every complete simulation step (not on every step iteration)
	 * use this function to implement custom reward functions that depend on additional metrics in the level
	 * @return agent reward 
	 */
	virtual float getReward(){return 0;}

	/* get goal
	 * @return Box2D fixture of the goal or NULL if goal does not exist
	 */
	b2Fixture* getGoal(){return (_goal == NULL) ? NULL : _goal->GetFixtureList();}

	/* get goal position
	 * @return current position of the goal or (0,0) if goal does not exist
	 */
	b2Vec2 getGoalPosition(){return (_goal ==  NULL) ? b2Vec2(0,0) : _goal->GetTransform().p;}

	/* check if robot had contact with a human
 	 * @return true if contact with human and false otherwise
	 */
	virtual bool checkHumanContact(b2Fixture * other_fixture){return false;}


protected:
	/* check whether goal position is such that robot does not touch it initially
	 * @return true if robot is does not touch the goal
	 * NOTE: this function only checks outer radii of goal and robot (not the actual shapes),
	 *		 thus it might be that on returning false, the robot still does not touch the goal
	 */
	bool checkValidGoalSpawn(const b2Vec2 & robot_pos, const b2Vec2 & spawn_pos){ 
		return (robot_pos-spawn_pos).Length() > (_SETTINGS->stage.goal_size/2.f+_levelDef.robot->getRadius());
	}
	
	/* spawns goal at a random position in the current goal spawn area
	 * if the goal position is not valid with respect to the robot_position (see checkValidGoalSpawn()) another random position is sampled and the check is performed again
	 * this is repeated until a valid position is found
	 * if after the 10th iteration no valid position was found, the goal is spawned regardless at the last position sampled
	 * @param goal_spawn if this parameter is NULL, the member RectSpawn _goalSpawnArea is used for sampling spawn positions,
	 					 else the RectSpawn pointed to by this parameter is used for sampling
	 */
	virtual void randomGoalSpawnUntilValid(RectSpawn * goal_spawn = NULL);


    /* spawns goal at a random position in the current goal spawn area
 * if the goal position is not valid with respect to the robot_position (see checkValidGoalSpawn()) another random position is sampled and the check is performed again
 * this is repeated until a valid position is found
 * if after the 10th iteration no valid position was found, the goal is spawned regardless at the last position sampled
 * @param goal_spawn if this parameter is NULL, the member RectSpawn _goalSpawnArea is used for sampling spawn positions,
                      else the RectSpawn pointed to by this parameter is used for sapling
 */
    bool obstacleSpawnUntilValid(RectSpawn *static_spawn, const std::list<b2Vec2*>& existing_positions, b2Vec2 &p);

    bool obstacleSpawnUntilValid(RectSpawn *static_spawn, const std::list<zRect*>& existing_boxes, b2Vec2 &p, int obstacle_type);

	/* create rectangular border around level origin (0,0) with given dimensions and add to body list
	 * @param half_width half the width (along x-axis) of border rect
	 * @param half_height half the height (along y-axis) of border rect
	 * @return static body
	 */
	b2Body* createBorder(float half_width, float half_height);

	/* create a circle as static obstacle and add to body list
	 * @param pos center position of circle
	 * @param radius radius of circle
	 * @return static body 
	 */
	b2Body* addCircle(const b2Vec2 & pos, float radius);

	/* create a rectangular box as static obstacle and add to body list
	 * @param pos center position of box
	 * @param half_width half width of box
	 * @param half_height half height of box
	 * @param angle angle in radians of box
	 * @return static body 
	 */
	b2Body* addBox(const b2Vec2 & center_pos, float half_width, float half_height, float angle = 0.f);

	/* create a custom Box2D shape as static obstacle and add to body list
	 * @param s shape to create static obstacle from
	 * @return static body 
	 */
	b2Body* addShape(const b2Shape * s);

	/* generate a randomly shaped static body and add to body list
	 * @param position center position of the shape
	 * @param min_radius minimum radius of the obstacle
	 * @param max_radius maximum radius of the obstacle
	 * @param aabb if not NULL this rect is set to the axis aligned bounding box enclosing the random obstacle
	 */
	b2Body* addRandomShape(const b2Vec2 & position, float min_radius, float max_radius, zRect * aabb = NULL);

	/* create multiple custom Box2D shapes as single static obstacle and add to body list
	 * @param shapes to create static obstacles from
	 * @return static body 
	 */
	b2Body* addShape(const std::vector<b2Shape*> shapes);

	/* reset robot position to (0,0) with random orientation
	 */
	void resetRobotToCenter(){_levelDef.robot->reset(b2Vec2_zero, f_frandomRange(0, 2*M_PI));}

	/* destroy all bodies in _bodyList and clear list, clears goal spawn area
	 */
	void clear();

	/* level initializer */
	LevelDef _levelDef;

	/* list of all bodies in level (e.g. obstacles), without the goal */
	std::list<b2Body*> _bodyList; 	

	/* body of goal */
	b2Body * _goal; 				

	/* spawn area of goal represented by multiple axis aligned rects*/
	RectSpawn _goalSpawnArea;
};

#endif
