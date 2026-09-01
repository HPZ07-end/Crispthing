package org.openbot.common;

import java.util.ArrayList;
import org.jetbrains.annotations.NotNull;
import org.openbot.R;
import org.openbot.model.Category;
import org.openbot.model.SubCategory;

public class FeatureList {
  // region Properties

  // Global
  public static final String ALL = "All";
  public static final String GENERAL = "General";
  public static final String LEGACY = "Legacy";
  public static final String DEFAULT = "Default";
  public static final String PROJECTS = "Projects";
  public static final String CONTROLLER = "Controller";
  public static final String CONTROLLER_MAPPING = "Controller Mapping";
  public static final String ROBOT_INFO = "Robot Info";

  // Game
  public static final String GAME = "Game";
  public static final String FREE_ROAM = "Free Roam";
  public static final String AR_MODE = "AR Mode";

  // Data Collection
  public static final String DATA_COLLECTION = "Data Collection";
  public static final String LOCAL_SAVE_ON_PHONE = "Local (save On Phone)";
  public static final String EDGE_LOCAL_NETWORK = "Edge (local Network)";
  public static final String CLOUD_FIREBASE = "Cloud (firebase)";
  public static final String CROWD_SOURCE = "Crowd-source (post/accept Data Collection Tasks)";

  // AI
  public static final String AI = "AI";
  public static final String AUTOPILOT = "Autopilot";
  public static final String PERSON_FOLLOWING = "Person Following";
  public static final String OBJECT_NAV = "指定人员跟随";
  public static final String MODEL_MANAGEMENT = "Model Management";
  public static final String POINT_GOAL_NAVIGATION = "Point Goal Navigation";
  public static final String AUTONOMOUS_DRIVING = "Autonomous Driving";
  public static final String VISUAL_GOALS = "Visual Goals";
  public static final String SMART_VOICE = "Smart Voice (left/right/straight, Ar Core)";

  // Remote Access
  public static final String REMOTE_ACCESS = "Remote Access";
  public static final String WEB_INTERFACE = "Web Interface";
  public static final String ROS = "ROS";
  public static final String FLEET_MANAGEMENT = "Fleet Management";

  // Coding
  public static final String CODING = "Coding";
  public static final String BLOCK_BASED_PROGRAMMING = "Block-Based Programming";
  public static final String SCRIPTS = "Scripts";

  // Research
  public static final String RESEARCH = "Research";
  public static final String CLASSICAL_ROBOTICS_ALGORITHMS = "Classical Robotics Algorithms";
  public static final String BACKEND_FOR_LEARNING = "Backend For Learning";

  // Monitoring
  public static final String MONITORING = "Monitoring";
  public static final String SENSORS_FROM_CAR = "Sensors from Car";
  public static final String SENSORS_FROM_PHONE = "Sensors from Phone";
  public static final String MAP_VIEW = "Map View";
  // endregion

  @NotNull
  public static ArrayList<Category> getCategories() {
    ArrayList<Category> categories = new ArrayList<>();

    ArrayList<SubCategory> subCategories = new ArrayList<>();

    // 当前首页只保留“指定人员跟随”一个入口
    subCategories.add(
            new SubCategory(
                    OBJECT_NAV,
                    R.drawable.ic_person_search,
                    "#E7CE88"
            )
    );

    categories.add(new Category(AI, subCategories));

    return categories;
  }
}
