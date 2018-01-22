import cv2
import numpy as np
from copy import deepcopy
import os
from time import sleep
from sklearn.neighbors import KNeighborsClassifier
from sklearn.model_selection import train_test_split
import sys
import pickle


cam_width = 0
cam_height = 0
min_size = 0
max_size = 0
categories = []

vc = cv2.VideoCapture()

# Debug level specifies a global debug information threshold
# level 0 shows only very important imformation
# level 1-2 are suitable when handling images in bulk
# level 3-5 are suitable when examinging one or a few images at a time
debug_detail_level = 2


# Closes all windows and ends the program
# reveives: N/A
# returns: N/A
def end_program():
    cv2.destroyAllWindows()
    global vc
    vc.release()
    sys.exit(0)


# This class remembers all the categories available
# This information is stored as:    integer format as indices
#                                   string format as 3 letter abbreviations
#                                   string format as a full description of the shape
# To use this class:    create an empty instance
#                           example: categories = Categories()
#                       call '.set_indices' supplying a range of numbers
#                           example: categories.set_indices([0, 1, 2, 3, 4])
#                       call '.set_abbreviations' with a list of available abbreviations
#                           example: categories.set_abbreviations(["CIR", "CRO", "SQU", "STA", "TRI"])
#                       call '.set_texts' with a list of available shape descriptions
#                           example: categories.set_texts(["Circle", "Cross", "Square", "Star", "Triangle"])
#                       once the class has been properly populated with the above methods,
#                           one can use it by calling one of it's main methods with a search argument
#                           in the form of a single integer or string
#                           then the functions return the desired equivalent value
#                           the methods in question are:
#                               '.return_index' searches for the supplied argument and returns it's index
#                               '.return_abbreviation' searches for the supplied argument and returns it's abbreviation
#                               '.return_text' searches for the supplied argument and returns it's text
#                           examples based on the above:    categories.return_index('CIR') would return 0
#                                                           categories.return.abbreviation(2) would return 'SQU'
#                                                           categories.return_text('TRI') would return 'Triangle'
class Categories():
    def __init__(self):
        self._indices = []
        self._abbreviations = []
        self._texts = []

    def set_indices(self, indices_arg):
        self._indices = indices_arg
    def get_indices(self):
        return self._indices

    def set_abbreviations(self, abbreviations_arg):
        self._abbreviations = abbreviations_arg
    def get_abbreviations(self):
        return self._abbreviations

    def set_texts(self, texts_arg):
        self._texts = texts_arg
    def get_texts(self):
        return self._texts

    def get_index(self, ind):
        return self.get_indices()[ind]
    def get_abbreviation(self, ind):
        return self.get_abbreviations()[ind]
    def get_text(self, ind):
        return self.get_texts()[ind]

    def return_index(self, input_arg):
        place = self.find_input_arg(input_arg)
        return self.get_index(place)
    def return_abbreviation(self, input_arg):
        place = self.find_input_arg(input_arg)
        return self.get_abbreviation(place)
    def return_text(self, input_arg):
        place = self.find_input_arg(input_arg)
        return self.get_text(place)

    # Sometimes a list of one integer needs to be changed to integer
    # receives input_arg <int or str>: a variable that may need trimming
    # returns output <int or str>: a variable in a trimmed state
    def trim_input_arg(self, input_arg):
        if type(input_arg) is list or type(input_arg) is tuple:
            output = input_arg[0]
        else:
            output = input_arg
        return output

    # Searches the 3 different arrays stored in this object and tries to find a match
    # receives input_arg <int or str>: the term that will be searched for
    # returns i <int>: if successful, it returns the position found
    def find_input_arg(self, input_arg):
        input_arg = self.trim_input_arg(input_arg)
        for i in range(len(self.get_indices())):
            if self.get_index(i) == input_arg:
                return i
            elif self.get_abbreviation(i) == input_arg:
                return i
            elif self.get_text(i) == input_arg:
                return i


        debug(1, "Did not recognise -", input_arg, "- as a valid value for category conversion.")
        end_program()

# Stores a contour and all relevant information about it
class Contour:
    def __init__(self, contour_arg=[]):
        self._contour = contour_arg
        self._shape_type = ""
        self._directory = ""
        self._filename = ""
        self._path = ""
        self._frame_number = ""
        self._feature_labels = []
        self._feature_data = []

        self._arcLength = ""
        self._contourArea = ""
        self._flimsiness = ""
        self._minAreaRect = ""
        self._minAreaRect_height = ""
        self._minAreaRect_width = ""
        self._minAreaRectArea = ""
        self._rect_aspect_ratio = ""
        self._minEnclosingCircle = ""
        self._hull = ""
        self._defects = ""
        self._hull_area = ""
        self._extent = ""
        self._solidity = ""

    def set_contour(self, contour_arg):
        self._contour = contour_arg
    def get_contour(self):
        return self._contour
    def set_frame_number(self, frame_number_arg):
        self._frame_number = frame_number_arg
    def get_frame_number(self):
        return self._frame_number
    def set_shape_type(self, shape_type_arg):
        if shape_type_arg:
            self._shape_type = shape_type_arg
        else:
            if self.get_filename():
                self._shape_type = self.get_filename()[0:3]
            else:
                self._shape_type = ""
    def get_shape_type(self):
        return self._shape_type
    def set_directory(self, directory_arg):
        self._directory = directory_arg
    def set_filename(self, filename_arg):
        self._filename = filename_arg
    def set_path(self, directory_arg, filename_arg):
        self._directory = directory_arg
        self._filename = filename_arg
    def get_path(self):
        self._path = self._directory + self._filename
        return self._path
    def get_directory(self):
        return self._directory
    def get_filename(self):
        return self._filename
    def set_frame_number(self, frame_number_arg):
        self._frame_number = frame_number_arg
    def get_frame_number(self, frame_number_arg):
        self._frame_number = frame_number_arg
    def set_contour(self, contour_arg):
        self._contour = contour_arg
    def set_feature_labels(self, feature_labels_arg=[]):
        if len(feature_labels_arg) < 1:
            self._feature_labels = ["shape_type", "flimsiness", "rect_aspect_ratio", "extent", "solidity"]
        else:
            self._feature_labels = feature_labels_arg
    def get_feature_labels(self):
        if len(self._feature_labels) < 1:
            self.set_feature_labels()
        return self._feature_labels
    def set_features(self):
        self.calculate_features()
        features_labels = self.get_feature_labels()
        feature_data = []
        for i in range(len(features_labels)):
            if features_labels[i] == "shape_type":
                feature_data.append(self.get_shape_type())
            elif features_labels[i] == "flimsiness":
                feature_data.append(self.get_flimsiness())
            elif features_labels[i] == "rect_aspect_ratio":
                feature_data.append(self.get_rect_aspect_ratio())
            elif features_labels[i] == "extent":
                feature_data.append(self.get_extent())
            elif features_labels[i] == "solidity":
                feature_data.append(self.get_solidity())
        self._feature_data = feature_data
    def get_features(self):
        return self._feature_data
    # Some of the following feature calculations are inspired by docs.opencv.org
    def calculate_features(self):
        self._arcLength = cv2.arcLength(self.get_contour(), True)
        self._contourArea = cv2.contourArea(self.get_contour())
        self._flimsiness = self.get_arcLength() / self.get_contourArea()
        self._minAreaRect = cv2.minAreaRect(self.get_contour())
        self._minAreaRect_height = self.get_minAreaRect()[1][0]
        self._minAreaRect_width = self.get_minAreaRect()[1][1]
        self._minAreaRectArea = self.get_minAreaRect_width() * self.get_minAreaRect_height()
        self._rect_aspect_ratio = self.get_minAreaRect_width() / self.get_minAreaRect_height()
        self._minEnclosingCircle = cv2.minEnclosingCircle(self.get_contour())
        self._hull = cv2.convexHull(self.get_contour(), returnPoints=False)
        self._defects = cv2.convexityDefects(self.get_contour(), self.get_hull())
        self._hull_area = cv2.contourArea(cv2.convexHull(self.get_contour()))
        self._extent = self.get_contourArea() / self.get_minAreaRectArea()
        self._solidity = self.get_contourArea() / self.get_hull_area()
        debug(3, "filename: ", self.get_filename())
        debug(4, "arclength: ", self.get_arcLength())
        debug(4, "contourArea: ", self.get_contourArea())
        debug(3, "flimsiness: ", self.get_flimsiness())
        debug(4, "minAreaRect: ", self.get_minAreaRect())
        debug(4, "minAreaRect_height: ", self.get_minAreaRect_height())
        debug(4, "minAreaRect_width: ", self.get_minAreaRect_width())
        debug(4, "minAreaRectArea: ", self.get_minAreaRectArea())
        debug(3, "rect_aspect_ratio: ", self.get_rect_aspect_ratio())
        debug(4, "minEnclosingCircle: ", self.get_minEnclosingCircle())
        debug(4, "hull: ", self.get_hull())
        debug(4, "defects: ", self.get_defects())
        debug(4, "hull_area: ", self.get_hull_area())
        debug(3, "extent: ", self.get_extent())
        debug(3, "solidity: ", self.get_solidity())
    def get_arcLength(self):
        return self._arcLength
    def get_contourArea(self):
        return self._contourArea
    def get_flimsiness(self):
        return self._flimsiness
    def get_minAreaRect(self):
        return self._minAreaRect
    def get_minAreaRect_height(self):
        return self._minAreaRect_height
    def get_minAreaRect_width(self):
        return self._minAreaRect_width
    def get_minAreaRectArea(self):
        return self._minAreaRectArea
    def get_rect_aspect_ratio(self):
        return self._rect_aspect_ratio
    def get_minEnclosingCircle(self):
        return self._minEnclosingCircle
    def get_hull(self):
        return self._hull
    def get_defects(self):
        return self._defects
    def get_hull_area(self):
        return self._hull_area
    def get_extent(self):
        return self._extent
    def get_solidity(self):
        return self._solidity

# Keeps and handles multiple contours
class Dataset:
    def __init__(self, task_arg=""):
        self._contour_list = []
        self._list_length_current = 0
        self._list_length_previous = 0
        self._shape_type_index = -1
        self._feature_indices = []
        self.data = []
        self.target = []
        self._categories = []
        self._task = task_arg

    def add_contour(self, new_contour_arg=Contour()):
        self._contour_list.append(new_contour_arg)
        self.list_length_increment()
    def list_length_increment(self):
        self._list_length_current += 1
    def update(self):
        if self._list_length_previous < self._list_length_current:
            self.set_categories()
            self.generate_new_dataset()
            self._list_length_previous = self._list_length_current
    def set_contour_feature_labels_range(self):
        lst = self.get_contour_list()[0].get_feature_labels()
        shape_type_index = -1
        feature_indices = []
        for i in range(len(lst)):
            if lst[i] == "shape_type" and shape_type_index == -1:
                shape_type_index = i
            else:
                feature_indices.append(i)
        self._shape_type_index = shape_type_index
        self._feature_indices = feature_indices
    def generate_new_dataset(self):
        self.check_dataset_length()
        self.set_contour_feature_labels_range()
        if self._task == "train":
            self.set_target()
        self.set_data()
    def get_contour_list(self):
        return self._contour_list
    def set_target(self):
        target_matrix = []
        global categories
        for t in range(len(self.get_contour_list())):
            curr_cont = self.get_contour_list()[t]
            target_line = curr_cont.get_features()[self._shape_type_index]
            target_matrix.append(categories.return_index(target_line))
        self.target = np.array(target_matrix)
    def set_data(self):
        data_matrix = []
        cont_list = self.get_contour_list()
        for c in range(len(cont_list)):
            data_line = []
            curr_cont = cont_list[c]
            for i in self._feature_indices:
                data_line.append(curr_cont.get_features()[i])
            data_matrix.append(data_line)
        self.data = np.array(data_matrix)
    def check_dataset_length(self):
        if len(self.get_contour_list()) < 1:
            debug(2, "Error: Dataset object cannot operate properly if it is empty.")
    def set_categories(self):
        global categories
        self._categories = categories


# This function prints debug messages if the message is important enough
# receives msg_detail_level_arg <int>: important the current debug message is
# receives arg0_arg <str>: a part of the message that will be concatenated to a combined string (optional)
# receives arg1_arg <str>: ditto
# receives arg2_arg <str>: ditto
# receives arg3_arg <str>: ditto
# receives arg4_arg <str>: ditto
# returns N/A
# Alternatively, this function can run other functions given by string value
#   in that case:
#       arg1_arg <str>: needs to be set to "func"
#       arg2_arg <str>: the name of the function to be called
#       arg3_arg <str>: the first argument for the function to be called
#       arg3_arg <str>: the second argument for the function to be called
def debug(msg_detail_level_arg=0, arg0_arg="", arg1_arg="", arg2_arg="", arg3_arg="", arg4_arg=""):
    if msg_detail_level_arg <= debug_detail_level:
        if arg0_arg == "func":
            if len(arg1_arg):
                function_to_call = globals()[arg1_arg]  # Get the function
                if arg2_arg:
                    if len(arg3_arg):
                        if arg4_arg:
                            function_to_call(arg2_arg, arg3_arg, arg4_arg)  # call it
                        else:
                            function_to_call(arg2_arg, arg3_arg)
                    else:
                        function_to_call(arg2_arg)
                return
            else:
                "Invalid use of debug function. Failed to use the debug function to call another function."
        else:
            print "Debug detail level " + str(msg_detail_level_arg) + " - " + str(arg0_arg) + str(arg1_arg) + str(arg2_arg) + str(arg3_arg) + str(arg4_arg)
            return


# Displays a preview of the current state of the image for debug purposes
# receives title_arg <str>: the title of the window that shows the image
# receives image_arg <image>: the image object to be shown
def preview_image(title_arg, image_arg):
    if not title_arg:
        title_arg = "Preview"
    cv2.imshow(title_arg, image_arg)
    cv2.waitKey(0)
    cv2.destroyWindow(title_arg)


# This class handles all classification for teaching and estimation purposes
# create an object without arguments to get an empty class of this type, then you can manipulate it via methods
# call the 'set_train' method to train the algorithm with locally stored images
# call the 'get_train' to return the trained algorithm
# call the 'contour_size_limits' method to set the minimum and maximum cought contour sizes
# call the 'cam' method starts webcam estimation
class Classifier():
    def __init__(self):
        self._k_nearest_neighbour = []
        self._min_size = 0
        self._max_size = 0
        self._contour_center = cam_width/2 , cam_height/2
        self._current_category = ""
        self._current_confidence = 0

    def _size_check(self):
        if self.get_contour_size_limits()[1] == 0:
            self.set_contour_size_limits()
    def set_train(self, new_arg=False, directories_arg=""):
        filename = 'knn_model.sav'
        if new_arg:
            self._size_check()
            if directories_arg == "":
                directories_arg = ["./teach_images/focus_good", "./teach_images/focus_bad"]
            self._k_nearest_neighbour = self._train_classifier(directories_arg)
            pickle.dump(self.get_train(), open(filename, 'wb'))
        else:
            self._k_nearest_neighbour = pickle.load(open(filename, 'rb'))


    def get_train(self):
        return self._k_nearest_neighbour


    def _train_classifier(self, directories_arg):
        dataset_teach = self._get_all_test_data(directories_arg)
        dataset_teach.update()

        X_all = dataset_teach.data
        y_all = dataset_teach.target
        X_train, X_test, y_train, y_test = train_test_split(X_all, y_all, test_size=0.33, random_state=42)
        knn = KNeighborsClassifier(15)
        knn.fit(X_train, y_train)
        prediction = knn.predict(X_test)

        right = 0
        global categories
        for test in range(len(prediction)):
            if categories.return_abbreviation([y_test[test]]) == categories.return_abbreviation([prediction[test]]):
                right += 1
        accuracy = str(float(right) / len(prediction))

        debug(1, "Training completed successfully with ", str(float(accuracy) * 100.0), "% accuracy.")
        return knn

    # Gets a list of paths for all images to be processed
    # receives directories_arg <str[]>: a list of strings describing the directories to grab data from
    # returns teach_dataset <Dataset>: an object that holds and handle multiple contours
    def _get_all_test_data(self, directories_arg):
        dirs, names, shapes = self._generate_image_paths(directories_arg)
        if len(names) < 1:
            debug(0, "Couldn't acquire data.", "Check whether the supplied directory is correct.")
            return

        nr_of_images_total = 0
        nr_of_images_kept = 0


        teach_dataset = Dataset("train")
        # for i in range (len(dirs)):
        for i in range(2000):
            image = cv2.imread(str(dirs[i]) + str(names[i]))
            debug(5, dirs[i], names[i])
            debug(4, "func", "preview_image", "Original image", image)
            cv2.waitKey(1)
            global cam_height
            global cam_width
            cam_height, cam_width, _ = image.shape
            self._size_check()
            image_processed = self._process_image(image)
            debug(4, "func", "preview_image", "Processed image", image_processed)
            cont, number = self._get_contours(image_processed)

            nr_of_images_total += 1
            if len(cont) == 1:
                new_contour = Contour(cont[0])
                new_contour.set_directory(dirs[i])
                new_contour.set_filename(names[i])
                new_contour.set_shape_type(shapes[i])
                new_contour.set_features()
                teach_dataset.add_contour(new_contour)
                nr_of_images_kept += 1
            else:
                debug(3, dirs[i], names[i], " failed to produce a contour and was discarded.")
            debug(4, "func", "preview_image", "Image with contours", image)

        debug(2, "For training purposes, ", nr_of_images_kept, " images kept out of ", nr_of_images_total)
        return teach_dataset


    # Pre-processes the image with certain tools in a specific order
    # receives image_arg <image>: the relative paths that you want to include in the search
    # returns funcionts_arg <str[]>: returns the directory of every file found
    # returns kernel_arg <int[][]>: returns the name of every file found
    def _process_image(self, image_arg, functions_arg=[], kernel_arg=""):
        if not functions_arg:
            functions_arg = ["gray", "increase_contrast", "open", "close"]
        if not kernel_arg:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        im = deepcopy(image_arg)
        im_tmp = deepcopy(image_arg)
        for f in functions_arg:
            if f == 'gray':
                im_tmp = cv2.cvtColor(deepcopy(im), cv2.COLOR_BGR2GRAY)
            if f == 'dilate':
                im_tmp = cv2.morphologyEx(deepcopy(im), cv2.MORPH_DILATE, kernel)
            elif f == 'erode':
                im_tmp = cv2.morphologyEx(deepcopy(im), cv2.MORPH_ERODE, kernel)
            elif f == 'open':
                im_tmp = cv2.morphologyEx(deepcopy(im), cv2.MORPH_OPEN, kernel)
            elif f == 'close':
                im_tmp = cv2.morphologyEx(deepcopy(im), cv2.MORPH_CLOSE, kernel)
            elif f == 'mean_shift':
                im_tmp = cv2.pyrMeanShiftFiltering(deepcopy(im), 31, 51)
            elif f == 'increase_contrast':
                im_tmp = cv2.convertScaleAbs(deepcopy(im), -1, 2, 0)
            elif f == 'decrease_contrast':
                im_tmp = cv2.convertScaleAbs(deepcopy(im), -1, 0.5, 0)
            else:
                pass
            im = deepcopy(im_tmp)
            debug(4, "func", "preview_image", str(f), im)
        th, im_thresh = cv2.threshold(im, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return im_thresh


    # Generates a list of paths of image files that can be used for processing
    # receives directories_arg <str[]>: the relative paths that you want to include in the search
    # returns dir_out <str[]>: returns the directory of every file found
    # returns file_out <str[]>: returns the name of every file found
    # returns figures_out <str[]>: the shape that should be displayed on that image
    def _generate_image_paths(self, directories_arg):
        dirs_out = []
        files_out = []
        figures_out = []
        starting_dir = os.getcwd()
        for directory in directories_arg:
            os.chdir(directory)
            for file in os.listdir(os.getcwd()):
                dirs_out.append(str(os.getcwd()) + "/")
                files_out.append(file)
                figures_out.append(file[0:3])

            os.chdir(starting_dir)
        return dirs_out, files_out, figures_out

    # Sets the contour size limits in percentage of image size
    def set_contour_size_limits(self, min_contour_size=0.005, max_contour_size=0.6):
        global max_size
        global min_size
        global cam_height
        global cam_width
        cam_size = cam_height * cam_width
        self._min_size = cam_size * min_contour_size
        self._max_size = cam_size * max_contour_size


    def get_contour_size_limits(self):
        return self._min_size, self._max_size


    # Gets the contours of an image
    # receives image_arg <image>: the image you want to extract contours from
    # receives min_size_arg <int>: used to determine the minimum size of a contour for it to be included (optional)
    # receives max_size_arg <int>: used to determine the maximum size of a contour for it to be included (optional)
    # returns conts_result <int[]>: returns the contours that have passed the editing process
    # returns number_of_conts <int>: returns the number of contours found
    def _get_contours(self, image_arg):
        try:
            _, conts, hierarchy = cv2.findContours(deepcopy(image_arg), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        except ValueError:
            conts, hierarchy = cv2.findContours(deepcopy(image_arg), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        debug(4, "Number of contours before tidying: ", len(conts))
        conts = self._fix_if_contour_is_touching_edge(conts, image_arg)
        conts_result = []
        sleep(0.01)
        if conts:
            for cont in conts:
                area = cv2.contourArea(cont)
                if area > self.get_contour_size_limits()[0]:
                    if area < self.get_contour_size_limits()[1]:
                        conts_result.append(cont)
                    else:
                        debug(5, "Contour removed because it was too big.")
                else:
                    debug(5, "Contour removed because it was too small.")
        debug(3, "Number of contours after tidying: ", len(conts_result))
        number_of_conts = len(conts_result)
        if len(conts_result) > 1:
            debug(3, "Multiple contours found.")
            debug(3, "func", "preview_image", "Multiple contours", image_arg)
        if len(conts_result) < 1:
            debug(3, "No contours found.")
            debug(3, "func", "preview_image", "No contours", image_arg)
        return conts_result, number_of_conts


    # In case there is a significant contour touching the border of the image, the two are separated with white pixels
    # receives contours_arg <int[]>: the contour list of an image
    # receives image_arg <image>: a thresholded image
    # returns conts <int[]>: in case a problem was found, this returns the new contour list
    def _fix_if_contour_is_touching_edge(self, contours_arg, image_arg):
        border_area = (cam_height - 3) * (cam_width - 3)
        border_length = 2 * (cam_height - 3) + 2 * (cam_width - 3)
        np_array = deepcopy(image_arg)
        conts = contours_arg
        last_cont = conts[len(conts)-1]
        rect = cv2.boundingRect(last_cont)
        area = cv2.contourArea(last_cont)
        length = cv2.arcLength(last_cont, True)
        extreme_corner_count = int(rect[0] <= 2) + int(rect[1] <= 2) + int(rect[2] >= (cam_width - 3))\
                               + int(rect[3] >= (cam_height - 3))
        if extreme_corner_count >= 3:
            if area < border_area - 1000 and length > border_length + 20:
                x_min = 0
                x_max = len(np_array[0]) - 1
                y_min = 0
                y_max = len(np_array) - 1
                thickness = 5
                colour = 255
                for x in range(x_min, x_max+1):
                    for i in range(thickness):
                        np_array[0+i, x] = colour
                        np_array[y_max-i, x] = colour
                for y in range(y_min, y_max+1):
                    for j in range(thickness):
                        np_array[y, 0+j] = colour
                        np_array[y, x_max-j] = colour
                try:
                    _, conts, h = cv2.findContours(deepcopy(np_array), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
                except ValueError:
                    conts, h = cv2.findContours(deepcopy(np_array), cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

                debug(4, "The image border was successfully separated from any intersecting contours.",
                      "This has increased the number of contours to: ", len(conts))
        return conts


    # Commences grabbing of webcam feed followed by feed classification
    # receives webcam_index_arg <int>: the index of the camera to use (optional)
    # returns: N/A
    def cam(self, webcam_index_arg=0):
        self._size_check()
        if not self.get_train():
            debug(1, "Error, no algorithm supplied to classifier function. Unable to classify camera feed.")
            end_program()
        self._classify_cam_feed(webcam_index_arg)


    # Captures video frames and sends it to processing, followed by contour estimation and contour marking
    # receives webcam_index_arg <int>: the index of the camera to use
    # returns: N/A
    def _classify_cam_feed(self, webcam_index_arg):

        rval = False
        cv2.namedWindow("preview")
        global vc
        vc = cv2.VideoCapture(webcam_index_arg)
        if vc.isOpened():  # try to get the first frame
            rval, frame = vc.read()
        else:
            rval = False

        global cam_height
        global cam_width
        cam_height, cam_width, _ = frame.shape
        self.set_contour_size_limits()

        interval = 1
        iteration = 0
        while rval:
            rval, frame = vc.read()  # read the image again at each loop
            key = cv2.waitKey(20)

            iteration += 1
            if iteration >= interval:
                self.classify_cam_frame(frame)
                iteration = 0
            if key == 27:  # exit on ESC
                end_program()



            cv2.imshow("preview", frame)
            cv2.waitKey(1)
    def classify_cam_frame(self, frame_arg):
        global cam_height
        global cam_width
        cam_height, cam_width, _ = frame_arg.shape
        self.set_contour_size_limits()
        live_dataset = self._process_frame(deepcopy(frame_arg))
        if not live_dataset == "":
            live_dataset.update()
            X = live_dataset.data
            prediction = self.get_train().predict(X)
            percentage = self.get_train().predict_proba(X)
            for d in range(len(live_dataset.get_contour_list())):
                self._mark_contour(frame_arg, live_dataset.get_contour_list()[d].get_contour(), prediction[d], percentage[d])





    # Processes the current video frame and turns it into contour
    # recieves frame_arg <int[]>: a raw frame to be processed
    # returns current_dataset <Dataset>: an objects that holds and handles multiple contours
    def _process_frame(self, frame_arg):


        frame_processed = self._process_image(frame_arg)
        #preview_image("aefv", frame_processed)
        conts, number = self._get_contours(frame_processed)
        current_dataset = Dataset("estimate")
        if number == 0:
            return ""
        for i in range(len(conts)):
            new_contour = Contour(conts[i])
            new_contour.set_features()
            current_dataset.add_contour(new_contour)
        return current_dataset


    def _mark_contour(self, frame_arg, contour_arg, prediction_arg, percentage_arg):
        global categories

        rect = cv2.boundingRect(contour_arg)
        self._set_contour_center(rect)
        x, y, w, h = rect
        cv2.rectangle(frame_arg, (x, y), (x + w, y + h), (0, 255, 0), 2)
        coor = (x, y)
        string_category = str(categories.return_abbreviation(prediction_arg))
        string_confidence = str(percentage_arg[prediction_arg] * 100)
        self._set_current_category(string_category)
        self._set_current_conficence(string_confidence)
        string_out = string_category + " (" + string_confidence + "%)"
        self._write_on_image(frame_arg, string_out, coor)

    def _set_current_category(self, current_category_arg):
        self._current_category = current_category_arg
    def get_current_category(self):
        return self._current_category
    def _set_current_conficence(self, current_confidence_arg):
        self._current_confidence = current_confidence_arg
    def get_current_confidence(self):
        return self._current_confidence

    def _set_contour_center(self, rectangle_arg):
        x, y, w, h = rectangle_arg
        x_center = x + w/2
        y_center = y + h/2
        self._contour_center = x_center, y_center


    def get_contour_center(self):
        return self._contour_center


    def _write_on_image(self, frame_arg=[], text_arg="", coordinate_arg="", colour_arg=[]):
        if len(frame_arg) < 1:
            debug(1, "No frame detected during write_on_image function call.")
            end_program()
        if text_arg == "":
            text_arg = "No text supplied"
        if len(coordinate_arg) < 1:
            text_arg = "No coordinates supplied"
            coordinate_arg = (10, 50)
        else:
            x_tmp, y_tmp = coordinate_arg
            y_tmp = y_tmp - 20
            if y_tmp < 50:
                y_tmp = 50
            coordinate_arg = x_tmp, y_tmp
        if len(colour_arg) < 1:
            debug(4, "No text colour supplied, reverting to default colour")
            colour_arg = (255, 0, 0)

        font = cv2.FONT_HERSHEY_SIMPLEX
        lower_left_corner = coordinate_arg
        fontScale = 1
        line_type = 2

        cv2.putText(frame_arg, text_arg, lower_left_corner, font, fontScale, colour_arg, line_type)




# Creates and populates a class that takes care of all conversions of category types
# receives ind_arg <int[]>: a list of indices to be used for categories
# receives abbr_arg <str[]>: a list of shape abbreviations to be used for categories
# receives text_arg <str[]>: a list of shape descriptions to be used for categories
# returns: N/A
def define_categories(ind_arg, abbr_arg, text_arg):
    global categories
    categories = Categories()
    categories.set_indices(ind_arg)
    categories.set_abbreviations(abbr_arg)
    categories.set_texts(text_arg)



                        # Down here is the main code.
                        # Everything else is just classes and functions

# Defining the available classification categories
define_categories(range(5), ["CIR", "CRO", "SQU", "STA", "TRI"], ["Circle", "Cross", "Square", "Star", "Triangle"])

# You can adjust this to get more or less debug feedback from the code
#   setting it to 0 returns only the most critical feedback
#   setting it to 5 gives you feedback in multiple loops throughout the code (not adviced)
#   setting it in between 0 and 5 gives amount of feedback relative to those extremes
debug_detail_level = 2

# Defines an empty classifier class
classifier = Classifier()
# Trains the classifier using locally stored images
# Pass in false to use prelearned model or true to relearn model
classifier.set_train(False)

# Shows the parameters for the currently trained classifier
#if debug_detail_level <= 1:
#    print classifier.get_train()
# Uses the trained classifier to evaluate a camera feed
#print("Starting camera feed evaluation.")
#print("Press 'esc' to end program")
#classifier.cam(1)
