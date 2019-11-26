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

import cv2
import rospy
import rospkg
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy

NAME = "CRUZ_NAVARRETE"

def evaluate(perceptron, image):
    #
    # TODO:
    # Calculate the output of perceptron given an image.
    # Use of numpy.dot is strongly recommended to save execution time.
    # Return perceptron output.
    #

    u = numpy.dot(perceptron,image)
    perceptron_output = 1 / (1+math.exp(-u))
    return perceptron_output

#training for number 1
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
    
    # initialized gradient
    grad = numpy.zeros(784) + [1]
    attemps = 0
    #
    tol = 0.001
    lis = []
    alpha = 0.00005
    max_attempts = 500
    
    #add -1 at the end  
    for i in range(len(dataset)):
        lis.append(numpy.asarray(dataset[i]+[-1]))

    perceptron = numpy.asarray(perceptron)
    
    #make sure the gradient norm is > than the tolerance 
    #limit the number of attempts
    while numpy.linalg.norm(grad) > tol and attemps < max_attempts:

        grad = numpy.zeros(785) #vector of zeros 

        for j in range(len(dataset)):

            y_hat = evaluate(perceptron, lis[j])
            #as we checked in the class 
            y_des = 1 if labels[j] == desired_digit else 0
            grad = numpy.add((y_hat-y_des)*(y_hat-y_hat*y_hat) * lis[j], grad)       
            perceptron = numpy.add(-alpha*grad, perceptron)

        print str(numpy.linalg.norm(grad)) + " Num.Attemps: " + str(attemps)
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
    img = testing_dataset[numpy.random.randint(0, 4999)]
    while not rospy.is_shutdown():
        test_digit = cv2.waitKey(10) - 48
        if test_digit >= 0 and test_digit <= 9:
            img = testing_dataset[test_digit*500 + numpy.random.randint(0, 499)]
            print "Perceptron output: " + str(evaluate(perceptron, img + [-1]))
        cv2.imshow("Digit", numpy.reshape(numpy.asarray(img, dtype="float32"), (28,28,1)))
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
