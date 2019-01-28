#!/usr/bin/env python

# Node for providing accurately timestmaped image messages.
#
# Subscribes to an image topic and a time reference topic.
# The job of the node is to match images with corresponding timestamps
# from the time reference topic.
#
# It is assumed that time reference messages preceed their corresponding
# images, so a given time reference will be matched with the next received
# image.



import rospy
from sensor_msgs.msg import TimeReference, Image
from std_msgs.msg import Float32

class TopicRateEstimator:
    """
    Responsible for determining the publish rate of a given topic.
    To use, provide with messages as they are received from a given topic.
    The object will provide a rolling estimate of the publish rate
    """

    def __init__(self, windowSize = 20):
        """
        Initialise with a window size - this is the number of messages which will 
        be assessed to estimate publish rate
        """
        self.windowSize = windowSize
        self.windowedTimetamps = [None] * windowSize

    def update(self, msg):
        self.windowedTimetamps.pop(0)
        self.windowedTimetamps.append(msg.header.stamp.to_sec())

    def getRate(self):
        windowStartTime = min(self.windowedTimetamps)
        windowEndTime = max(self.windowedTimetamps)

        if windowStartTime is None:
            rate = None
        else:
            windowTimeSpan = windowEndTime - windowStartTime
            rate = (self.windowSize-1) / windowTimeSpan 

        return rate
        
        

class TimestampMatcher:
    """
    Responsible for receiving images and time references from incoming topics, and 
    returning images with timestamps updated to reference values
    """

    def __init__(self):
        self.latestImage = None
        self.latestTimestamp = None
        self.estimatedFrameRate = None
        self.rateEstimator = TopicRateEstimator()
        self.rateEstimate = None
        self.initialised = False #used to disable some warnings generated as matcher is initialising
        rospy.logwarn('ignoring images until capture rate is known')

    def receiveTimeReference(self, msg):
        #update header stamp with time received (can't trust arduino to provide stamps in sync with system clock)
        msg.header.stamp = rospy.get_rostime()

        if self.latestTimestamp is not None and self.initialised:
            #receiveImg will set 'latestTimestamp' back to none,
            #so if we receive a new reference while latestTimestamp is not none,
            #we must have missed an image
            rospy.logwarn('received reference timestamp without a corresonding image')
        self.latestTimestamp = msg
    
    def receiveImg(self, msg):
        """
        Receive image from incoming topic, and if possible, return it with the
        corrected timestamp
        """

        #first task is to determine the image publish rate
        if self.rateEstimate is None:
            self.rateEstimator.update(msg)
            self.rateEstimate = self.rateEstimator.getRate()

            if self.rateEstimate is not None:
                self.initialised = True
                rospy.logwarn('Capture rate is determined ({h} hz). Begining timestamp matching'.format(h=self.rateEstimate))            
            return None
        
        #now determine whether it is possible to correct the given image timestamp
        if self.latestTimestamp is None:
            rospy.logwarn('ignoring image: no timestamp reference available (reference timestamper probably missed a frame)')
            return None
        
        #to guard against dropped timestamps, only match a timestamp with an image 
        #if the time difference is within a certain fraction of the capture rate
        rateFractionThreshold = 0.8
        timeOffsetThreshold = (1/self.rateEstimate) * rateFractionThreshold
        refTimeToMsgTimeOffset = (msg.header.stamp - self.latestTimestamp.header.stamp).to_sec() 
        if refTimeToMsgTimeOffset > timeOffsetThreshold:
            rospy.logwarn('ignoring image: delay between time reference and image is too large (delay = {d}, threshold = {t})'.format(
                d=refTimeToMsgTimeOffset, t=timeOffsetThreshold)
            )
            return None
            
        #all ready to match timestamp with image. Do this and return the result
        updatedImage = msg

        initialTimestamp = msg.header.stamp
        updatedImage.header.stamp = self.latestTimestamp.time_ref
        self.latestTimestamp = None #prevent reuse of the same timestamp

        return updatedImage
        


def timestamp_matcher():

    rospy.init_node('timestamp_matcher')

    timestampMatcher = TimestampMatcher()

    imgPub = rospy.Publisher('image_raw_timestamp_corrected', Image, queue_size=10)

    #option: set publish_corrections true to publish the timestamp correction sizes to
    #a float topic
    publishCorrections = rospy.get_param('~publish_corrections', False)
    if publishCorrections:
        correctionPub = rospy.Publisher('timestamp_corrections', Float32, queue_size=10)


    def publishUpdatedImage(msg):
        initialImgStamp = msg.header.stamp

        updatedImg = timestampMatcher.receiveImg(msg)

        if updatedImg is not None:

            if publishCorrections:
                #out of interest - by how much is the timestamp being corrected? Is it
                #a constant offset or a varying amount?
                correctionSize = (updatedImg.header.stamp - initialImgStamp).to_sec()
                correctionPub.publish(Float32(correctionSize))
            

            imgPub.publish(msg)

    rospy.Subscriber("image_raw", Image, publishUpdatedImage)
    rospy.Subscriber("image_time_reference", TimeReference, timestampMatcher.receiveTimeReference)

    rospy.spin()

if __name__ == '__main__':
    timestamp_matcher()
