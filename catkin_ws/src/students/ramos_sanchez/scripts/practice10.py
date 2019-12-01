#!/usr/bin/env python
#
# AUTONOMOUS MOBILE ROBOTS - UNAM, FI, 2020-1
# PRACTICE 10 - PERCEPTRON TRAINING BY GRADIENT RULE
#
# Instructions:
# Complete the code to train a perceptron using the gradient rule.
# Perceptron's output should indicate wheter a given input image
# corresponds to the trained digit or not. 
#

import numpy as np
import cv2
import math
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError

NAME = "Ramos_Sanchez"

def evaluate(perceptron, image):
    #
    # TODO:
    # Calculate the output of perceptron given an image.
    # Use of numpy.dot is strongly recommended to save execution time.
    # Return perceptron output.
    #
    per = np.dot(perceptron, image)
    per_output = 1 / (1 + math.exp(-per))
    return per_output

def train_perceptron(perceptron, dataset, labels, desired_digit):
    print "Training perceptron for digit " + str(desired_digit) + " with " + str(len(dataset)) + " images. "
    #
    # TODO:
    # Train perceptron given a set of training images (dataset), a set of labels (corresponding digits)
    # and a desired digit to be trained.
    # Perceptron is an array of floats representing the input weights.
    # Last value in the array corresponds to threshold.
    # You can use gradient rule or perceptron rule, but gradient rule is recommended.
    # Use of numpy.dot, numpy.add, numpy.zeros, numpy.asarray and numpy.linalg.norm is suggested.
    # Return the trained perceptron.
    #
    grad = np.zeros(784) + [1]
    attemps = 0
    tol = 0.001
    list = []
    alpha = 0.00005
    maxAttemps = 500

    for i in range(len(dataset)):
        list.append(np.asarray(dataset[i] + [-1]))
    perceptron = np.asarray(perceptron)

    while np.linalg.norm(grad) > tol and attemps < maxAttemps:
        grad = np.zeros(785)
        for j in range(len(dataset)):
            y_hat = evaluate(perceptron, list[j])
            y_des = 1 if labels[j] == desired_digit else 0
            grad = np.add((y_hat - y_des) * (y_hat - y_hat * y_hat) * list[j], grad)
            perceptron = np.add(-alpha * grad, perceptron)
        print str(np.linalg.norm(grad)) + " number of attemp: " + str(attemps)
        attemps += 1
    return perceptron

def load_dataset_digit(file_name):
    #It is assumed the file contains 1000 images with 28x28 grayscale pixels each one.
    print "Loading data set from file " +  file_name
    f_data = [ord(c)/255.0 for c in open(file_name, "rb").read(784000)]
    images = []
    for i in range(1000):
        images.append(f_data[784*i:784*(i+1)])
    print "Loaded " + str(len(images)) + " images."
    return images

def load_dataset_all_digits(folder):
    print "Loading data set from " + folder
    if not folder.endswith("/"):
        folder += "/"
    training_dataset = []
    training_labels  = []
    testing_dataset  = []
    testing_labels   = []
    for i in range(10):
        digit_dataset = load_dataset_digit(folder + "data" + str(i))
        training_dataset += digit_dataset[0:len(digit_dataset)/2]
        training_labels  += [i for j in range(len(digit_dataset)/2)]
        testing_dataset  += digit_dataset[len(digit_dataset)/2:len(digit_dataset)]
        testing_labels   += [i for j in range(len(digit_dataset)/2)]
    return training_dataset, training_labels, testing_dataset, testing_labels

def main():
    print "PRACTICE 10 - " + NAME
    rospy.init_node("practice10")
    rospack = rospkg.RosPack()
    dataset_folder = rospack.get_path("datasets") + "/handwriting_digits/"
    if rospy.has_param("~dataset_folder"):
        dataset_folder = rospy.get_param("~dataset_folder")
    desired_digit = 0
    if rospy.has_param("~digit"):
        desired_digit = rospy.get_param("~digit")

    training_dataset, training_labels, testing_dataset, testing_labels = load_dataset_all_digits(dataset_folder)
    perceptron = [0 for i in range(784+1)]
    perceptron = train_perceptron(perceptron, training_dataset, training_labels, desired_digit)
    
    loop = rospy.Rate(10)
    img = testing_dataset[np.random.randint(0, 4999)]
    while not rospy.is_shutdown():
        test_digit = cv2.waitKey(10) - 48
        if test_digit >= 0 and test_digit <= 9:
            img = testing_dataset[test_digit*500 + np.random.randint(0, 499)]
            print "Perceptron output: " + str(evaluate(perceptron, img + [-1]))
        cv2.imshow("Digit", np.reshape(np.asarray(img, dtype="float32"), (28,28,1)))
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

