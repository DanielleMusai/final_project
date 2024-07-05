import pygame
import sys
import time
import random

# from pygame.draw_py import Point
from pygame.locals import QUIT, KEYDOWN, K_UP, K_DOWN, K_LEFT, K_RIGHT

from point import Point

# Initialize Pygame
pygame.init()

# Constants
PIXELS_PER_CM = 2.5
DETECTION_RANGE_CM = 300  # 300
DETECTION_RANGE_PX = int(DETECTION_RANGE_CM / PIXELS_PER_CM)
DRONE_RADIUS_CM = 10
DRONE_RADIUS_PX = int(DRONE_RADIUS_CM / PIXELS_PER_CM)
SENSOR_RATE = 10  # 10 times per second
BATTERY_LIFE_MINUTES = 4
BATTERY_LIFE_SECONDS = BATTERY_LIFE_MINUTES * 60
DRONE_SPEED_CM_PER_SEC = 100  # 100 # 1 meter per second
DRONE_SPEED_PX_PER_SEC = int(DRONE_SPEED_CM_PER_SEC / PIXELS_PER_CM)
PATH_HISTORY = []
POINT_HISTORY = []
ENDLESS_LOOP = []
LATS_FIVE_MOVES = []

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
YELLOW = (255, 255, 0)
TEXT_COLOR = (255, 255, 255)

# Global variables for detection distances
detect_distance_up = float('inf')
detect_distance_down = float('inf')
detect_distance_right = float('inf')
detect_distance_left = float('inf')

"""
TO DO
1 adjust the distance the drone can be from the wall !! now it can reach to close 
2 movement algo 
3 GREEN point position
4 percentage of map that has been seen   
5 make sure that all the numbers are correct 10 times per second and all of that !
6 see the assignment directions if more things are needed to be done !
7 change names to match assignment
8 add pitch and roll
"""

"""
Function to load and display the map
this is the main loop that displays the map 
call functions 
and waits fro movement directions 
"""

def find_closest_revisited_point(current_pos, path_history):
    point_counts = {}
    for pos in path_history:
        if tuple(pos) in point_counts:
            point_counts[tuple(pos)] += 1
        else:
            point_counts[tuple(pos)] = 1

    revisited_points = [pos for pos, count in point_counts.items() if count > 1]
    if not revisited_points:
        return -1  # No revisited points

    closest_point_index = -1
    closest_distance = float('inf')
    for i, pos in enumerate(path_history):
        if tuple(pos) in revisited_points:
            distance = ((current_pos[0] - pos[0]) ** 2 + (current_pos[1] - pos[1]) ** 2) ** 0.5
            if distance < closest_distance:
                closest_distance = distance
                closest_point_index = i

    return closest_point_index

def display_map(image_path, drone_pos_cm):
    map_image = pygame.image.load(image_path)  # Load the image
    map_width, map_height = map_image.get_size()  # Get the size of the image in pixels
    screen = pygame.display.set_mode((map_width, map_height))  # Create a window with the size of the image
    pygame.display.set_caption('Map Viewer')

    drone_pos_px = [drone_pos_cm[0], drone_pos_cm[1]]

    start_time = time.time()  # Start time
    font = pygame.font.Font(None, 24)  # Font for displaying text

    # calculate the closest wall for start
    direction = closest_wall_direction(screen, drone_pos_px)
    following_wall_direction = direction
    main_movment_direction = direction
    # Main loop to keep the window open
    prev_drone_pos = None
    count = 0  # this is for flag 1
    count2 = 0  # this is for flag 2
    false_corner = False
    flag = False
    flag2 = False

    return_journey = False  # Flag to indicate the return journey
    return_index = -1  # Index for the return path
    pygame.init()
    screen = pygame.display.set_mode((1500, 600))
    while True:
        # print("wall direction = " + str(following_wall_direction))
        # print("movment direction = " + str(main_movment_direction))
        dist_dict = {(0, -1): detect_distance_up, (0, 1): detect_distance_down, (-1, 0): detect_distance_left,
                     (1, 0): detect_distance_right}  # up down right left
        current_time = time.time()
        elapsed_time = current_time - start_time
        time_remaining = max(0, BATTERY_LIFE_SECONDS - elapsed_time)
        minutes_remaining = int(time_remaining // 60)
        seconds_remaining = int(time_remaining % 60)

        # if time_remaining == 0:
        #     print("Battery life depleted. Exiting...")
        #     break

        if time_remaining <= (BATTERY_LIFE_SECONDS / 2) + 1 and not return_journey:
            print("Initiating return journey...")
            # print(f"PATH_HISTORY: {PATH_HISTORY}")
            return_journey = True
            return_index = find_closest_revisited_point(drone_pos_px, PATH_HISTORY)

        if return_journey:
            flag = False
            flag2 = False
            false_corner = False

            if return_index >= 0:
                prev_drone_pos = drone_pos_px
                drone_pos_px = PATH_HISTORY[return_index]
                return_index = return_index - 1

                screen.blit(map_image, (0, 0))  # Draw the image onto the screen
                draw_drone_detect_and_color(screen, drone_pos_px, map_image,
                                            direction)  # Draw the drone and its detection range on the map
                draw_text(screen, f"Time Remaining: {minutes_remaining:02d}:{seconds_remaining:02d}", font, TEXT_COLOR,
                          (10, 10))  # Draw the time remaining
                pygame.display.update()  # Update the display
            else:
                print("Drone has returned to the starting point.")
                # break  # End the simulation when the drone returns home
        else:
            if flag == False:
                # we first start with potential movements for the wall and main direction to see where we can go
                potential_position_wall = move_drone(drone_pos_px, map_image, following_wall_direction)
                potential_position_main_movment = move_drone(drone_pos_px, map_image, main_movment_direction)
                # print("flag if")

                # this is for flag 2 explanation is further down
                if count2 == 3:
                    flag2 = False
                    count2 = 0
                    # print("flag2 if")

                # if flag2 is on befor going back to the main algorithem we want to make afue more moves in the main movment direction
                if (potential_position_wall is not None) and (potential_position_main_movment is not None) and flag2:
                    prev_drone_pos = drone_pos_px
                    drone_pos_px = potential_position_main_movment
                    direction = main_movment_direction
                    count2 = count2 + 1
                    # print("if 1")

                # this means i changed direction to a place i cant go to
                elif flag2 and potential_position_main_movment is None:
                    # switch back directions
                    save = main_movment_direction  # old wall
                    main_movment_direction = [following_wall_direction[0] * -1, following_wall_direction[1] * -1]
                    following_wall_direction = save
                    flag2 = False
                    # print("if 2")
                    continue

                #################################################
                # for the most part this is the main algorithm of the movement when all is good !!!

                # if i can go to the wall go
                elif potential_position_wall is not None and false_corner == False:
                    prev_drone_pos = drone_pos_px
                    drone_pos_px = potential_position_wall
                    direction = following_wall_direction
                    # print("if 3")

                # if not the wall try new movment direction
                elif (potential_position_wall is None) and (
                        potential_position_main_movment is not None) and false_corner == False:
                    prev_drone_pos = drone_pos_px
                    drone_pos_px = potential_position_main_movment
                    direction = main_movment_direction
                    # print("if 4")

                # if i cant got ither way and its the start
                elif (potential_position_wall is None) and (potential_position_main_movment is None) and (
                        following_wall_direction == main_movment_direction) and false_corner == False:
                    main_movment_direction = direction_change(main_movment_direction)
                    # print("if 5")
                    continue

                # this hopefully is when i am in a corner
                elif (potential_position_wall is None) and (potential_position_main_movment is None) and (
                        dist_dict[tuple(following_wall_direction)] < 40) and (
                        dist_dict[tuple(main_movment_direction)] < 40) and false_corner == False:
                    # print("corner")
                    following_wall_direction = main_movment_direction
                    continue

                # this is when i detected a new space but i am to close to the wall so i cant move to that direction yet
                elif (potential_position_wall is None) and (potential_position_main_movment is None) and (
                        (dist_dict[tuple(following_wall_direction)] > 20) or (
                        dist_dict[tuple(main_movment_direction)] > 20)) and false_corner == False:
                    false_corner = True
                    # print("false corner")

                #################################################

            ########################################
            # when flag is on this means we detected a radical change in the distance from the wall we are following flag is here to correctly handel this
            # move 2 steps in the main movement direction before changing directions
            # then flag2 will also help with afue moves to a new specific direction and finally we will turn them False to continue normaly
            if flag:
                if count == 2:
                    count = 0
                    flag = False
                    # print("if 6")

                    # print("changing direction")

                    save = following_wall_direction
                    following_wall_direction = [main_movment_direction[0] * -1, main_movment_direction[1] * -1]
                    main_movment_direction = save
                    continue
                else:
                    if one_more_move(drone_pos_px, map_image, main_movment_direction) is not None:
                        prev_drone_pos = drone_pos_px
                        drone_pos_px = one_more_move(drone_pos_px, map_image, main_movment_direction)
                        direction = main_movment_direction
                        count = count + 1
                        # print("if 7")
                    else:
                        opesit_wall = [following_wall_direction[0] * -1, following_wall_direction[1] * -1]
                        potential = move_drone(drone_pos_px, map_image, opesit_wall)
                        if potential is None:
                            # print("if 8")
                            flag = False
                            flag2 = False
                            following_wall_direction = opesit_wall
                            main_movment_direction = [main_movment_direction[0] * -1, main_movment_direction[1] * -1]
                            continue
                        else:
                            prev_drone_pos = drone_pos_px
                            drone_pos_px = potential
                            direction = opesit_wall
                            # print("if 9")

            # there are instances of false corners where we detect smace but cant move yet due to the size of the drone this part will correct the movemaet acordinly.
            if false_corner:
                opesit_wall_direction = [following_wall_direction[0] * -1, following_wall_direction[1] * -1]
                potential_opesit_wall = move_drone(drone_pos_px, map_image, opesit_wall_direction)
                if potential_opesit_wall is not None:
                    prev_drone_pos = drone_pos_px
                    drone_pos_px = potential_opesit_wall
                    direction = opesit_wall_direction
                    # print("if 10")

                if potential_position_main_movment is not None:
                    prev_drone_pos = drone_pos_px
                    drone_pos_px = potential_position_main_movment
                    direction = main_movment_direction
                    false_corner = False
                    # print("if 11")

                if potential_opesit_wall is None and potential_position_main_movment is None:
                    opesit_main = [main_movment_direction[0] * -1, main_movment_direction[1] * -1]
                    potential_opesit_main = move_drone(drone_pos_px, map_image, opesit_main)
                    # print("if 12")
                    if potential_opesit_main is not None:
                        prev_drone_pos = drone_pos_px
                        drone_pos_px = potential_opesit_main
                        direction = opesit_main

                        following_wall_direction = main_movment_direction
                        main_movment_direction = opesit_wall_direction
                        false_corner = False
                        # print("if 13")

            # to prevent unnecessary movements towards the wall this basically says if you are close enough and there is nothing stoping you go in the main movement direction not the wall
            if dist_dict[tuple(main_movment_direction)] == float('inf') and dist_dict[
                tuple(following_wall_direction)] < 30 and flag == False and false_corner == False and potential_position_main_movment is not None and direction != main_movment_direction:
                # print("if 14")
                if drone_pos_px != potential_position_main_movment:
                    prev_drone_pos = drone_pos_px
                    drone_pos_px = potential_position_main_movment
                    direction = main_movment_direction
                    # print("if 15")

            # I think this is pretty self-explanatory
            if drone_pos_px == prev_drone_pos:
                print("changed pos equal")
                save = following_wall_direction
                following_wall_direction = [main_movment_direction[0] * -1, main_movment_direction[1] * -1]
                main_movment_direction = save
                continue

            prev_dist_wall = dist_dict[tuple(following_wall_direction)]
            screen.blit(map_image, (0, 0))  # Draw the image onto the screen
            # print("prev_drone_pos: = " + str(prev_drone_pos[0]) + " , " + str(prev_drone_pos[1]))
            # print("drone_pos_px: = " + str(drone_pos_px[0]) + " , " + str(drone_pos_px[1]))
            draw_drone_detect_and_color(screen, drone_pos_px, map_image,
                                        direction)  # Draw the drone and its detection range on the map
            PATH_HISTORY.append(drone_pos_px)  # Record the position
            draw_text(screen, f"Time Remaining: {minutes_remaining:02d}:{seconds_remaining:02d}", font, TEXT_COLOR,
                      (10, 10))  # Draw the time remaining
            pygame.display.update()  # Update the display

            if endless_loop(drone_pos_px):
                save = following_wall_direction
                following_wall_direction = main_movment_direction
                main_movment_direction = save
                ENDLESS_LOOP.clear()

            # this is very important not to move because the radical change function has variables that get updated after we draw_drone_detect_and_color !!!!!
            temp = radical_change(prev_dist_wall, following_wall_direction, direction)
            if temp:
                flag = True  # this is so i can move
                flag2 = True  # this is for the main movement so i dont go back from where i came
                # print("flag True")

        """
        this may need updating or deleting
        """
        time.sleep(1 / SENSOR_RATE)  # Sleep to simulate the sensor update rate


'''
if we detect a radical change in the wall direction before changing directions we will use this function to do afue more moves so that we wont bump it to the wall
'''


def one_more_move(pos, map, main_movment):
    dist_dict = {(0, -1): detect_distance_up, (0, 1): detect_distance_down, (-1, 0): detect_distance_left,
                 (1, 0): detect_distance_right}
    if dist_dict[tuple(main_movment)] >= 20:
        location = move_drone(pos, map, main_movment)
        return location
    return None


'''
this function will detect a radicl change in the distance from the wall we are following
'''


def radical_change(last_wall_dist, wall_direction, movment_direction):
    dist_dict = {(0, -1): detect_distance_up, (0, 1): detect_distance_down, (-1, 0): detect_distance_left,
                 (1, 0): detect_distance_right}  # up down right left
    if (((last_wall_dist * 3) - 10 < dist_dict[tuple(wall_direction)]) or (
            dist_dict[tuple(wall_direction)] >= 110 and last_wall_dist <= 70)) and wall_direction[0] != \
            movment_direction[0]:  # we lost the wall direction went != wall direction
        # following_wall_direction = [main_movment_direction[0] * -1, main_movment_direction[1] * -1]
        return True
    return False


'''
now we need a function that will take us in a different direction after reaching the nearest wall 
ideally to the place where we detect more space not less !
'''


def direction_change(old_direction):
    directions = [
        (0, -1),  # Up
        (0, 1),  # Down
        (1, 0),  # Right
        (-1, 0)  # Left
    ]
    direction = []
    # we went up down
    if old_direction[0] == 0:
        if detect_distance_left > detect_distance_right:
            # we go left
            direction = [-1, 0]
        else:
            # we go right
            direction = [1, 0]
    # we went left right
    if old_direction[1] == 0:
        if detect_distance_up > detect_distance_down:
            # we go up
            direction = [0, -1]
        else:
            # we go down
            direction = [0, 1]
    return direction


'''
for the initial movement we will use this function to detect the closest wall to follow
'''


def closest_wall_direction(screen, drone_pos_px):
    directions = [
        (0, -1),  # Up
        (0, 1),  # Down
        (1, 0),  # Right
        (-1, 0)  # Left
    ]

    min_dist = float('inf')
    direction = (0, 0)

    for dx, dy in directions:
        for i in range(1, DETECTION_RANGE_PX + 1):
            x = drone_pos_px[0] + dx * i
            y = drone_pos_px[1] + dy * i
            if 0 <= x < screen.get_width() and 0 <= y < screen.get_height():
                color = screen.get_at((int(x), int(y)))
                if color == BLACK:
                    dist = i
                    if dist < min_dist:
                        min_dist = dist
                        direction = [dx, dy]
                    break

    return direction


'''
this code needs to be adjusted to only move the drone and not validate the position 
'''


def move_drone(current_pos_px, map_image, movement_direction):
    map_width, map_height = map_image.get_size()
    dx = movement_direction[0]
    dy = movement_direction[1]
    if (dx == 0):
        new_x = current_pos_px[0]
    else:
        new_x = current_pos_px[0] + ((DRONE_SPEED_PX_PER_SEC / SENSOR_RATE) * dx)
    if (dy == 0):
        new_y = current_pos_px[1]
    else:
        new_y = current_pos_px[1] + ((DRONE_SPEED_PX_PER_SEC / SENSOR_RATE) * dy)
    new_pos_px = [new_x, new_y]

    if 0 <= new_pos_px[0] < map_width and 0 <= new_pos_px[1] < map_height:
        if validate_and_adjust_position(new_pos_px, map_image, map_width, map_height):
            return new_pos_px
    return None


'''
this returns true or false if the new position is good 
'''


def validate_and_adjust_position(drone_pos_px, map_image, map_width, map_height):
    safe_distance_px = int(20 / PIXELS_PER_CM)

    new_x, new_y = int(drone_pos_px[0]), int(drone_pos_px[1])
    blocking_direction = None  # Initialize to None, indicating no blockage yet

    # Ensure the new position is within map boundaries
    if not (0 <= new_x < map_width and 0 <= new_y < map_height):
        return False

    # Get the surrounding area within the safe distance
    for dx in range(-safe_distance_px, safe_distance_px + 1):
        for dy in range(-safe_distance_px, safe_distance_px + 1):
            check_x = new_x + dx
            check_y = new_y + dy
            if 0 <= check_x < map_width and 0 <= check_y < map_height:
                color = map_image.get_at((check_x, check_y))
                if color == BLACK:
                    return False
                    # # Calculate the direction of the blockage
                    # if dx < 0:
                    #     blocking_direction = "left"
                    # elif dx > 0:
                    #     blocking_direction = "right"
                    # elif dy < 0:
                    #     blocking_direction = "up"
                    # elif dy > 0:
                    #     blocking_direction = "down"
                    # return False, blocking_direction

    color = map_image.get_at((new_x, new_y))
    # this already returns true or false
    return color == WHITE or color == YELLOW


"""
Function to detect the map area and color it yellow
this needs to happen 10 times per second !!

here we can do a test before coloring yellow if its already yellow we where here !!!! 
"""


def draw_drone_detect_and_color(screen, drone_pos_px, map_image, movment_direction):
    was_i_here = False  # this will be returned True if the detected era was yellow before

    global detect_distance_up, detect_distance_down, detect_distance_left, detect_distance_right
    center_x, center_y = int(drone_pos_px[0]), int(drone_pos_px[1])
    pygame.draw.circle(screen, RED, (center_x, center_y), DRONE_RADIUS_PX)
    # Draw the points from POINT_HISTORY
    for point in POINT_HISTORY:
        pygame.draw.circle(screen, GREEN, (point.x, point.y), DRONE_RADIUS_PX)

    directions = [
        (0, -DETECTION_RANGE_PX),  # Forward (up)
        (0, DETECTION_RANGE_PX),  # Backward (down)
        (DETECTION_RANGE_PX, 0),  # Right
        (-DETECTION_RANGE_PX, 0)  # Left
    ]

    # Count the number of infinite directions
    inf_count = 0
    inf_directions = []

    for dx, dy in directions:
        detected_distance = float('inf')
        for i in range(1, DETECTION_RANGE_PX + 1):
            x = center_x + dx * i // DETECTION_RANGE_PX
            y = center_y + dy * i // DETECTION_RANGE_PX

            if 0 <= x < screen.get_width() and 0 <= y < screen.get_height():
                color = screen.get_at((x, y))
                if i == DETECTION_RANGE_PX and color == WHITE:
                    inf_count += 1
                    inf_directions.append((dx, dy))
                if color == YELLOW:
                    was_i_here = True
                elif color == WHITE:
                    was_i_here = False
                    screen.set_at((x, y), YELLOW)
                    map_image.set_at((x, y), YELLOW)
                elif color == BLACK:
                    detected_distance = i
                    break

        if dx == 0 and dy < 0:
            detect_distance_up = detected_distance
        elif dx == 0 and dy > 0:
            detect_distance_down = detected_distance
        elif dx > 0 and dy == 0:
            detect_distance_right = detected_distance
        elif dx < 0 and dy == 0:
            detect_distance_left = detected_distance

    if inf_count >= 2:
        Point_displacement(screen, center_x, center_y, inf_directions, movment_direction)

    return was_i_here


'''
this is a feature that is not yet used this potentially puts a point on the map where the drone detected more than 2 optional directions and didnt explore one of them yet

implementing when we create a point we will only put inf where we did not map yet from this point so the movement direction or the direction we came from even if the distance is infinit we will still disregard it   

this now finds relevent places to go back to it dose not find all the spots i would like it to find but for now lets worck with what we have 
'''


def Point_displacement(screen, center_x, center_y, inf_directions,
                       movment_direction):  # fix do calculations in detect and color !!!!!
    # Assuming the drone's current position is the center of the point
    new_point = Point(center_x, center_y)
    add = False

    # Update the infinite directions in the Point object
    for dx, dy in inf_directions:
        if (dx == 0 and dy < 0) and not ((movment_direction[0] == 0 and movment_direction[1] < 0) or (
                movment_direction[0] == 0 and movment_direction[1] > 0)):  # if inf is up direction cant be up or down
            new_point.inf_front = True
            add = True

        elif (dx == 0 and dy > 0) and not ((movment_direction[0] == 0 and movment_direction[1] > 0) or (
                movment_direction[0] == 0 and movment_direction[1] < 0)):  # if inf is down direction cant be down or up
            new_point.inf_back = True
            add = True

        elif (dx > 0 and dy == 0) and not ((movment_direction[0] > 0 and movment_direction[1] == 0) or (
                movment_direction[0] < 0 and movment_direction[
            1] == 0)):  # if inf is right direction cant be right or left
            new_point.inf_right = True
            add = True

        elif (dx < 0 and dy == 0) and not ((movment_direction[0] < 0 and movment_direction[1] == 0) or (
                movment_direction[0] > 0 and movment_direction[
            1] == 0)):  # if inf is left direction cant be left or right
            new_point.inf_left = True
            add = True

    # Add the Point object to POINT_HISTORY
    if add:
        POINT_HISTORY.append(new_point)
        print("New Point:", new_point)
    # pygame.draw.circle(screen, GREEN, (center_x, center_y), DRONE_RADIUS_PX)


"""
Function to draw text on the screen
and may be deleted and simply moved to the main loop in display_map 
"""


def draw_text(screen, text, font, color, position):
    text_surface = font.render(text, True, color)
    screen.blit(text_surface, position)


'''
this function will get a 2 points and will calculate if point 2 is close to point 1 in a given era
 -----------
|         2 |      
|     1     |
|           |
 -----------
 this may look big here but we will test a very small margin 

 for every point in our saved path we will check if it is in the locations range if there is only one then that is easy but if there are more that means that there may be a better path to that location  
'''
# def point_2_in_1(screen,location,point)
'''
this will find an endless loop that it within an x amount of moves 
'''


def endless_loop(new_pos):
    for i in range(len(ENDLESS_LOOP)):
        if new_pos[0] == ENDLESS_LOOP[i][0] and new_pos[1] == ENDLESS_LOOP[i][1]:
            if len(ENDLESS_LOOP) - i > 5:
                return True

    if len(ENDLESS_LOOP) == 50:
        for j in range(len(ENDLESS_LOOP) - 1):
            ENDLESS_LOOP[j] = ENDLESS_LOOP[j + 1]
        ENDLESS_LOOP[len(ENDLESS_LOOP) - 1] = new_pos
        return None

    # if we did not find the point we add it and return None
    ENDLESS_LOOP.append(new_pos)
    return None


# Main function
if __name__ == "__main__":
    # Path to the image file
    image_path = "/Users/edenmor/Downloads/robots_1-main/Maps/p11.png"  # change as needed !!!!

    # Load the map image to get its dimensions
    map_image = pygame.image.load(image_path)
    map_width, map_height = map_image.get_size()


    # Function to generate a random position within the map
    def get_random_position():
        while True:
            x = random.randint(0, map_width - 1)
            y = random.randint(0, map_height - 1)
            if map_image.get_at((x, y)) == WHITE:  # Ensure the position is on a white pixel (free space)
                return (x, y)


    if map_image.get_at((110, 80)) == WHITE:
        drone_position_cm = (110, 80)  # Example position
    else:
        # Generate a random initial position for the drone
        drone_position_cm = get_random_position()

    display_map(image_path, drone_position_cm)

