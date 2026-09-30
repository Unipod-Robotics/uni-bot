cmake_minimum_required(VERSION 3.8)
project(ubot_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch missions maps_gt scripts
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

# Third-party models are fetched, not committed (scripts/fetch_models.sh).
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/models_external)
  install(DIRECTORY models_external/ DESTINATION share/${PROJECT_NAME}/models)
else()
  message(WARNING "ubot_worlds: models_external/ missing - run scripts/fetch_models.sh; "
                  "only the arena world will load.")
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
