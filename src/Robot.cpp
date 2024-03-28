#include "Robot.hpp"

#include "Client.hpp"
#include "CommunicationService.hpp"
#include "Goal.hpp"
#include "Logger.hpp"
#include "MainApplication.hpp"
#include "MathUtils.hpp"
#include "Message.hpp"
#include "MessageTypes.hpp"
#include "RobotWorld.hpp"
#include "Server.hpp"
#include "Shape2DUtils.hpp"
#include "Wall.hpp"
#include "WayPoint.hpp"

#include <chrono>
#include <cmath>
#include <ctime>
#include <sstream>
#include <thread>
#include <random>
#include <string>
#include <iostream>

namespace Model {
/**
 *
 */
Robot::Robot() :
		Robot("", wxDefaultPosition) {
}
/**
 *
 */
Robot::Robot(const std::string &aName) :
		Robot(aName, wxDefaultPosition) {
}
/**
 *
 */
Robot::Robot(const std::string &aName, const wxPoint &aPosition) :
		name(aName), size(wxDefaultSize), position(aPosition), front(0, 0), speed(
				0.0), acting(false), driving(false), communicating(false), merged(
				false), robotType(NONE) {
	// We use the real position for starters, not an estimated position.
	startPosition = position;
}
/**
 *
 */
Robot::~Robot() {
	if (driving) {
		Robot::stopDriving();
	}
	if (acting) {
		Robot::stopActing();
	}
	if (communicating) {
		stopCommunicating();
	}
}
/**
 *
 */
void Robot::setName(const std::string &aName,
		bool aNotifyObservers /*= true*/) {
	name = aName;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}
/**
 *
 */
wxSize Robot::getSize() const {
	return size;
}
/**
 *
 */
void Robot::setSize(const wxSize &aSize, bool aNotifyObservers /*= true*/) {
	size = aSize;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}
/**
 *
 */
void Robot::setPosition(const wxPoint &aPosition,
		bool aNotifyObservers /*= true*/) {
	position = aPosition;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}
/**
 *
 */
BoundedVector Robot::getFront() const {
	return front;
}
/**
 *
 */
void Robot::setFront(const BoundedVector &aVector,
		bool aNotifyObservers /*= true*/) {
	front = aVector;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}
/**
 *
 */
float Robot::getSpeed() const {
	return speed;
}
/**
 *
 */
void Robot::setSpeed(float aNewSpeed, bool aNotifyObservers /*= true*/) {
	speed = aNewSpeed;
	if (aNotifyObservers == true) {
		notifyObservers();
	}
}
/**
 *
 */
void Robot::startActing() {
	acting = true;
	std::thread newRobotThread([this] {
		startDriving();
	});
	robotThread.swap(newRobotThread);
}
/**
 *
 */
void Robot::stopActing() {
	acting = false;
	driving = false;
	robotThread.join();
}
/**
 *
 */
void Robot::startDriving() {
	driving = true;
	goal = RobotWorld::getRobotWorld().getGoal("Goal");
	calculateRoute(goal);

	drive();
}
/**
 *
 */
void Robot::stopDriving() {
	driving = false;
}
/**
 *
 */
void Robot::startCommunicating() {
	if (!communicating) {
		communicating = true;

		std::string localPort = "12345";
		if (Application::MainApplication::isArgGiven("-local_port")) {
			localPort =
					Application::MainApplication::getArg("-local_port").value;
		}

		if (Messaging::CommunicationService::getCommunicationService().isStopped()) {
			TRACE_DEVELOP("Restarting the Communication service");
			Messaging::CommunicationService::getCommunicationService().restart();
		}

		server = std::make_shared<Messaging::Server>(
				static_cast<unsigned short>(std::stoi(localPort)),
				toPtr<Robot>());
		Messaging::CommunicationService::getCommunicationService().registerServer(
				server);
	}
}
/**
 *
 */
void Robot::stopCommunicating() {
	if (communicating) {
		communicating = false;

		std::string localPort = "12345";
		if (Application::MainApplication::isArgGiven("-local_port")) {
			localPort =
					Application::MainApplication::getArg("-local_port").value;
		}

		Messaging::Client c1ient("localhost",
				static_cast<unsigned short>(std::stoi(localPort)),
				toPtr<Robot>());
		Messaging::Message message(Messaging::StopCommunicatingRequest, "stop");
		c1ient.dispatchMessage(message);
	}
}
/**
 *
 */
wxRegion Robot::getRegion() const {
	wxPoint translatedPoints[] = { getFrontRight(), getFrontLeft(),
			getBackLeft(), getBackRight() };
	return wxRegion(4, translatedPoints); // @suppress("Avoid magic numbers")
}
/**
 *
 */
bool Robot::intersects(const wxRegion &aRegion) const {
	wxRegion region = getRegion();
	region.Intersect(aRegion);
	return !region.IsEmpty();
}
/**
 *
 */
wxPoint Robot::getFrontLeft() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	wxPoint originalFrontLeft(x, y);
	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	wxPoint frontLeft(
			static_cast<int>((originalFrontLeft.x - position.x)
					* std::cos(angle)
					- (originalFrontLeft.y - position.y) * std::sin(angle)
					+ position.x),
			static_cast<int>((originalFrontLeft.y - position.y)
					* std::cos(angle)
					+ (originalFrontLeft.x - position.x) * std::sin(angle)
					+ position.y));

	return frontLeft;
}
/**
 *
 */
wxPoint Robot::getFrontRight() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	wxPoint originalFrontRight(x + size.x, y);
	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	wxPoint frontRight(
			static_cast<int>((originalFrontRight.x - position.x)
					* std::cos(angle)
					- (originalFrontRight.y - position.y) * std::sin(angle)
					+ position.x),
			static_cast<int>((originalFrontRight.y - position.y)
					* std::cos(angle)
					+ (originalFrontRight.x - position.x) * std::sin(angle)
					+ position.y));

	return frontRight;
}
/**
 *
 */
wxPoint Robot::getBackLeft() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	wxPoint originalBackLeft(x, y + size.y);

	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	wxPoint backLeft(
			static_cast<int>((originalBackLeft.x - position.x) * std::cos(angle)
					- (originalBackLeft.y - position.y) * std::sin(angle)
					+ position.x),
			static_cast<int>((originalBackLeft.y - position.y) * std::cos(angle)
					+ (originalBackLeft.x - position.x) * std::sin(angle)
					+ position.y));

	return backLeft;
}
/**
 *
 */
wxPoint Robot::getBackRight() const {
	// x and y are pointing to top left now
	int x = position.x - (size.x / 2);
	int y = position.y - (size.y / 2);

	wxPoint originalBackRight(x + size.x, y + size.y);

	double angle = Utils::Shape2DUtils::getAngle(front) + 0.5 * Utils::PI;

	wxPoint backRight(
			static_cast<int>((originalBackRight.x - position.x)
					* std::cos(angle)
					- (originalBackRight.y - position.y) * std::sin(angle)
					+ position.x),
			static_cast<int>((originalBackRight.y - position.y)
					* std::cos(angle)
					+ (originalBackRight.x - position.x) * std::sin(angle)
					+ position.y));

	return backRight;
}
/**
 *
 */
void Robot::handleNotification() {
	//	std::unique_lock<std::recursive_mutex> lock(robotMutex);

	static int update = 0;
	if ((++update % 200) == 0) // @suppress("Avoid magic numbers")
			{
		notifyObservers();
	}
}
/**
 *
 */
void Robot::handleRequest(Messaging::Message &aMessage) {
	FUNCTRACE_TEXT_DEVELOP(aMessage.asString());

	switch (aMessage.getMessageType()) {
	case Messaging::StopCommunicatingRequest: {
		aMessage.setMessageType(Messaging::StopCommunicatingResponse);
		aMessage.setBody("StopCommunicatingResponse");
		// Handle the request. In the limited context of this works. I am not sure
		// whether this works OK in a real application because the handling is time sensitive,
		// i.e. 2 async timers are involved:
		// see CommunicationService::stopServer and Server::stopHandlingRequests
		Messaging::CommunicationService::getCommunicationService().stopServer(
				12345, true); // @suppress("Avoid magic numbers")

		break;
	}
	case Messaging::EchoRequest: {
		aMessage.setMessageType(Messaging::EchoResponse);
		aMessage.setBody("Messaging::EchoResponse: " + aMessage.asString());
		break;
	}
	case Messaging::MergeRequest: {
		if (!merged) {
			aMessage.setMessageType(Messaging::MergeResponse);
			aMessage.setBody("Let's do some merging: " + aMessage.asString());
			merged = true;
			robotType = SLAVE;
			Application::Logger::log("I will act as the slave robot");
		}
		break;
	}
	case Messaging::RobotLocationRequest: {
		if (merged) {
			this->updateOtherRobot(aMessage.getBody());
		}

		aMessage.setMessageType(Messaging::RobotLocationResponse);

		std::ostringstream os;
		os << position.x << " " << position.y << " " << getFront().asString();
		aMessage.setBody(os.str());
		break;
	}

	default: {
		TRACE_DEVELOP(
				__PRETTY_FUNCTION__ + std::string(": default not implemented"));
		break;
	}
	}
}
/**
 *
 */
void Robot::handleResponse(const Messaging::Message &aMessage) {
	FUNCTRACE_TEXT_DEVELOP(aMessage.asString());

	switch (aMessage.getMessageType()) {
	case Messaging::StopCommunicatingResponse: {
		//Messaging::CommunicationService::getCommunicationService().stop();
		break;
	}
	case Messaging::EchoResponse: {
		break;
	}
	case Messaging::MergeResponse: {
		if (!merged) {
			merged = true;
			robotType = MASTER;
			Application::Logger::log("This robot will act as the master");
			this->askForLocation();
		}
		break;
	}
	case Messaging::RobotLocationResponse: {
		if (merged) {
			this->updateOtherRobot(aMessage.getBody());
		}
		break;
	}
	default: {
		TRACE_DEVELOP(
				__PRETTY_FUNCTION__ + std::string(": default not implemented, ")
				+ aMessage.asString());
		break;
	}
	}
}
/**
 *
 */
std::string Robot::asString() const {
	std::ostringstream os;

	os << "Robot " << name << " at (" << position.x << "," << position.y << ")";

	return os.str();
}
/**
 *
 */
std::string Robot::asDebugString() const {
	std::ostringstream os;

	os << "Robot:\n";
	os << "Robot " << name << " at (" << position.x << "," << position.y
			<< ")\n";

	return os.str();
}
/**
 *
 */
void Robot::drive() {
	try {
		// The runtime value always wins!!
		speed =
				static_cast<float>(Application::MainApplication::getSettings().getSpeed());

		// Compare a float/double with another float/double: use epsilon...
		if (std::fabs(speed - 0.0) <= std::numeric_limits<float>::epsilon()) {
			setSpeed(10.0, false); // @suppress("Avoid magic numbers")
		}

		Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": Drive the MF to goal"));

		// We use the real position for starters, not an estimated position.
		startPosition = position;

		WayPointPtr getOutOfMyWayPoint =
				Model::RobotWorld::getRobotWorld().newWayPoint("WP",
						wxPoint(600, 600));
		bool goingToWayPoint = false;

		// 1 or -1
		signed char direction = 1;

		unsigned short pathPoint = 0;
		while (position.x > 0 && position.x < 500 && position.y > 0
				&& position.y < 500 && pathPoint < path.size()) // @suppress("Avoid magic numbers")
		{
			// Do the update
			if (pathPoint >= path.size()) {
				pathPoint = 0;
			}
			const PathAlgorithm::Vertex &vertex = path[pathPoint];
			std::cout << "Before calc " << pathPoint << std::endl;
			pathPoint += static_cast<unsigned short>(speed * direction);
			std::cout << "After calc " << pathPoint << std::endl;

			front = BoundedVector(vertex.asPoint(), position);
			direction = 1;
			position.x = vertex.x;
			position.y = vertex.y;
			std::ostringstream os;
			os << "butter is at x: " << position.x;
			os << " and at y: " << position.y;

			Application::Logger::log(os.str());

			if (merged) {
				this->askForLocation();
				if (this->otherRobotOnPath(pathPoint)) {
					Application::Logger::log(
							__PRETTY_FUNCTION__
									+ std::string(": I see the other robot in my way"));
					if (this->toCloseToWall()) {
						if (this->robotType == SLAVE) {
							direction = -1;
						}
					} else {
						driving = false;
						signed short x = 0;
						if (speed != 0) {
							x = static_cast<signed short>(position.x
									+ 80 * (front.y / speed));
						} else {
							x = static_cast<signed short>(position.x + 80);
						}
						signed short y = static_cast<signed short>(position.y);

						std::ostringstream os;

						os << front.x << " " << front.y;

						getOutOfMyWayPoint->setPosition(wxPoint(x, y));

						pathPoint = 0;
						calculateRoute(getOutOfMyWayPoint);
						goingToWayPoint = true;

						driving = true;
					}
				} else if (this->otherRobotWithinRadius(60)) {
					Application::Logger::log(
							__PRETTY_FUNCTION__
									+ std::string(": I see the other robot in my way"));

					if (robotType == SLAVE) {
						Application::Logger::log(
								"I have low priority");
						std::this_thread::sleep_for(
								std::chrono::milliseconds(2000));
					} else {
						Application::Logger::log(
								"I have high priority");
					}

				}
			}

			// Stop on arrival or collision
			if (arrived(goal)) {
				Application::Logger::log(
						__PRETTY_FUNCTION__ + std::string(": arrived"));
				driving = false;
			} else if (goingToWayPoint && arrived(getOutOfMyWayPoint)) {
				Application::Logger::log(
						__PRETTY_FUNCTION__
								+ std::string(": arrived at waypoint"));
				std::cout << "Arrived at waypoint " << std::endl;
				driving = false;

				Application::Logger::log(
						__PRETTY_FUNCTION__
								+ std::string(": calculating route"));
				pathPoint = 0;
				direction = 1;
				goingToWayPoint = false;
				std::cout << "Editing waypoint" << std::endl;
				getOutOfMyWayPoint->setPosition(wxPoint(600, 600));
				std::cout << "Edited waypoint" << std::endl;
				std::cout << "Calculating route" << std::endl;

				calculateRoute(goal);
				std::cout << "Calculated route" << std::endl;

				Application::Logger::log(
						__PRETTY_FUNCTION__
								+ std::string(": calculated route"));

				driving = true;
			} else if (collision()) {
				Application::Logger::log(
						__PRETTY_FUNCTION__
								+ std::string(": Oh no, the robot has crashed"));
				driving = false;
			}

			notifyObservers();

			// If there is no sleep_for here the robot will immediately be on its destination....
			std::this_thread::sleep_for(std::chrono::milliseconds(100)); // @suppress("Avoid magic numbers")

			// this should be the last thing in the loop
			if (driving == false) {
				break;
			}
		} // while
	} catch (std::exception &e) {
		Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": ") + e.what());
		std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
	} catch (...) {
		Application::Logger::log(
				__PRETTY_FUNCTION__ + std::string(": unknown exception"));
		std::cerr << __PRETTY_FUNCTION__ << ": unknown exception" << std::endl;
	}
}
/**
 *
 */
void Robot::calculateRoute(GoalPtr aGoal) {
	path.clear();
	if (aGoal) {
		// Turn off logging if not debugging AStar
		Application::Logger::setDisable();

		front = BoundedVector(aGoal->getPosition(), position);
		//handleNotificationsFor( astar);
		path = astar.search(position, aGoal->getPosition(), size);
		//stopHandlingNotificationsFor( astar);

		Application::Logger::setDisable(false);
	}
}

void Robot::calculateRoute(WayPointPtr aWayPoint) {
	path.clear();
	if (aWayPoint) {
		// Turn off logging if not debugging AStar
		Application::Logger::setDisable();

		front = BoundedVector(aWayPoint->getPosition(), position);
		//handleNotificationsFor( astar);
		path = astar.search(position, aWayPoint->getPosition(), size);
		//stopHandlingNotificationsFor( astar);

		Application::Logger::setDisable(false);
	}
}
/**
 *
 */
bool Robot::arrived(GoalPtr aGoal) {
	if (aGoal && intersects(aGoal->getRegion())) {
		return true;
	}
	return false;
}

bool Robot::arrived(WayPointPtr aWaypoint) {
	if (aWaypoint && intersects(aWaypoint->getRegion())) {
		return true;
	}
	return false;
}
/**
 *
 */
bool Robot::collision() {
	wxPoint frontLeft = getFrontLeft();
	wxPoint frontRight = getFrontRight();
	wxPoint backLeft = getBackLeft();
	wxPoint backRight = getBackRight();

	const std::vector<WallPtr> &walls = RobotWorld::getRobotWorld().getWalls();
	for (WallPtr wall : walls) {
		if (Utils::Shape2DUtils::intersect(frontLeft, frontRight,
				wall->getPoint1(), wall->getPoint2())
				|| Utils::Shape2DUtils::intersect(frontLeft, backLeft,
						wall->getPoint1(), wall->getPoint2())
				|| Utils::Shape2DUtils::intersect(frontRight, backRight,
						wall->getPoint1(), wall->getPoint2())) {
			return true;
		}
	}
	const std::vector<RobotPtr> &robots =
			RobotWorld::getRobotWorld().getRobots();
	for (RobotPtr robot : robots) {
		if (getObjectId() == robot->getObjectId()) {
			continue;
		}
		if (intersects(robot->getRegion())) {
			return true;
		}
	}
	return false;
}

void Robot::askForLocation() {
	Model::RobotPtr robot = Model::RobotWorld::getRobotWorld().getRobot(
			"Butter");

	if (!robot) {
		return;
	}

	std::ostringstream os;
	os << __PRETTY_FUNCTION__ << " asking for location" << std::endl;

	std::string remoteIpAdres = "localhost";
	std::string remotePort = "12345";

	if (Application::MainApplication::isArgGiven("-remote_ip")) {
		remoteIpAdres =
				Application::MainApplication::getArg("-remote_ip").value;
	}
	if (Application::MainApplication::isArgGiven("-remote_port")) {
		remotePort = Application::MainApplication::getArg("-remote_port").value;
	}

	Application::Logger::log(os.str());
	Messaging::Client client(remoteIpAdres,
			static_cast<unsigned short>(std::stoi(remotePort)), robot);

	std::ostringstream location;
	location << position.x << " " << position.y << " " << getFront().asString();

	client.dispatchMessage(
			Messaging::Message(Messaging::RobotLocationRequest,
					location.str()));
}

// checks whether the remote robot is within given radius. Used for checking an imminent colission
bool Robot::otherRobotWithinRadius(signed long radius) {
	RobotPtr butterTheSecond = Model::RobotWorld::getRobotWorld().getRobot(
			"Peanut");

	if (!butterTheSecond)
		return false;

	return Utils::Shape2DUtils::distance(this->getFrontLeft(),
			butterTheSecond->getPosition()) < radius
			|| Utils::Shape2DUtils::distance(this->getFrontRight(),
					butterTheSecond->getPosition()) < radius;
}

void Robot::updateOtherRobot(std::string otherMsgBody) {
	std::stringstream is(otherMsgBody);
	signed short x, y, bvx, bvy;
	is >> x >> y >> bvx >> bvy;

	RobotPtr butterTheSecond = Model::RobotWorld::getRobotWorld().getRobot(
			"Peanut");
	if (!butterTheSecond) {
		Model::RobotWorld::getRobotWorld().newRobot("Peanut", wxPoint(x, y));
		butterTheSecond = Model::RobotWorld::getRobotWorld().getRobot("Peanut");
	}

	butterTheSecond->setPosition(wxPoint(x, y));

	BoundedVector b(bvx, bvy);
	butterTheSecond->setFront(b);

	Application::Logger::log(otherMsgBody);
}

bool Robot::otherRobotOnPath(unsigned short pathPoint) {
	RobotPtr butterTheSecond = Model::RobotWorld::getRobotWorld().getRobot(
			"Peanut");
	if (!butterTheSecond) {
		return false;
	}
	for (unsigned short vertexNr = pathPoint; vertexNr < pathPoint + 200;
			vertexNr++) {
		if (static_cast<unsigned long long>(vertexNr) + 1 >= path.size()) {
			return false;
		}

		if (Utils::Shape2DUtils::intersect(butterTheSecond->getFrontLeft(),
				butterTheSecond->getFrontRight(),
				wxPoint(path[vertexNr].x, path[vertexNr].y),
				wxPoint(path[vertexNr + 1].x, path[vertexNr + 1].y))
				|| Utils::Shape2DUtils::intersect(
						butterTheSecond->getFrontLeft(),
						butterTheSecond->getBackLeft(),
						wxPoint(path[vertexNr].x, path[vertexNr].y),
						wxPoint(path[vertexNr + 1].x, path[vertexNr + 1].y))
				|| Utils::Shape2DUtils::intersect(
						butterTheSecond->getFrontRight(),
						butterTheSecond->getBackRight(),
						wxPoint(path[vertexNr].x, path[vertexNr].y),
						wxPoint(path[vertexNr + 1].x, path[vertexNr + 1].y))
				|| Utils::Shape2DUtils::intersect(
						butterTheSecond->getBackLeft(),
						butterTheSecond->getBackRight(),
						wxPoint(path[vertexNr].x, path[vertexNr].y),
						wxPoint(path[vertexNr + 1].x, path[vertexNr + 1].y))) {
			return true;
		}
	}
	return false;
}


bool Robot::toCloseToWall() {

	RobotPtr butterTheSecond = Model::RobotWorld::getRobotWorld().getRobot(
			"Peanut");
	if (!butterTheSecond) {
		return false;
	}
	std::vector<WallPtr> Walls = Model::RobotWorld::getRobotWorld().getWalls();
	for (WallPtr wall : Walls) {
		if (Utils::Shape2DUtils::isOnLine(wall->getPoint1(), wall->getPoint2(),
				getFrontLeft(),
				static_cast<int>(Utils::Shape2DUtils::distance(getFrontLeft(),
						getFrontRight())))) {
			return true;
		}
		if (Utils::Shape2DUtils::isOnLine(wall->getPoint1(), wall->getPoint2(),
				getFrontRight(),
				static_cast<int>(Utils::Shape2DUtils::distance(getFrontLeft(),
						getFrontRight())))) {
			return true;
		}
	}
	return false;
}

bool Robot::getMerged() {
	return this->merged;
}

} // namespace Model
