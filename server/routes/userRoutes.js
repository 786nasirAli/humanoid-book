const express = require('express');
const router = express.Router();
const { MongoClient, ObjectId } = require('mongodb');
require('dotenv').config();

// Connect to MongoDB
const mongoClient = new MongoClient(process.env.MONGODB_URL);
let db;

async function initializeDb() {
  try {
    await mongoClient.connect();
    db = mongoClient.db(process.env.MONGODB_DATABASE);
    console.log('Connected to MongoDB');
  } catch (error) {
    console.error('Error connecting to MongoDB:', error);
  }
}

// Initialize database connection
initializeDb();

// Get user profile by ID
router.get('/user/:userId', async (req, res) => {
  try {
    const userId = req.params.userId;
    
    // Query user from MongoDB
    const userCollection = db.collection('users');
    const user = await userCollection.findOne({ _id: new ObjectId(userId) });
    
    if (!user) {
      return res.status(404).json({ error: 'User not found' });
    }
    
    // Return user profile data
    const userProfile = {
      id: user._id.toString(),
      name: user.name,
      email: user.email,
      softwareExperience: user.softwareExperience || '',
      hardwareExperience: user.hardwareExperience || '',
      roboticsBackground: user.roboticsBackground || '',
      learningGoals: user.learningGoals || '',
      preferredLanguage: user.preferredLanguage || 'english',
      createdAt: user.createdAt,
      lastActive: user.lastActive || new Date().toISOString()
    };
    
    res.status(200).json(userProfile);
  } catch (error) {
    console.error('Error fetching user profile:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
});

// Update user profile
router.put('/user/profile', async (req, res) => {
  try {
    const { userId, preferences } = req.body;
    
    // Update user in MongoDB
    const userCollection = db.collection('users');
    const result = await userCollection.updateOne(
      { _id: new ObjectId(userId) },
      { 
        $set: { 
          preferences: preferences,
          lastUpdated: new Date().toISOString()
        }
      }
    );
    
    if (result.matchedCount === 0) {
      return res.status(404).json({ error: 'User not found' });
    }
    
    res.status(200).json({
      success: true,
      message: 'Profile updated successfully',
      preferences
    });
  } catch (error) {
    console.error('Error updating user profile:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
});

// Save user background information
router.post('/user/background', async (req, res) => {
  try {
    const backgroundData = req.body;
    
    // Update user in MongoDB with background information
    const userCollection = db.collection('users');
    const result = await userCollection.updateOne(
      { _id: new ObjectId(backgroundData.userId) },
      {
        $set: {
          softwareExperience: backgroundData.softwareExperience,
          hardwareExperience: backgroundData.hardwareExperience,
          roboticsBackground: backgroundData.roboticsBackground,
          learningGoals: backgroundData.learningGoals,
          preferredLanguage: backgroundData.preferredLanguage
        }
      }
    );
    
    if (result.matchedCount === 0) {
      return res.status(404).json({ error: 'User not found' });
    }
    
    res.status(200).json({
      success: true,
      message: 'Background information saved successfully'
    });
  } catch (error) {
    console.error('Error saving background info:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
});

// Get user preferences for specific content
router.get('/user/preferences/:contentId', async (req, res) => {
  try {
    const { contentId } = req.params;
    
    // In a real implementation, this would fetch from the database
    // For now, return default preferences
    const defaultPrefs = {
      difficultyLevel: 'medium',
      explanationStyle: 'detailed',
      contentFormat: 'text',
      language: 'english'
    };
    
    res.status(200).json(defaultPrefs);
  } catch (error) {
    console.error('Error fetching preferences:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
});

// Save/update user preferences for specific content
router.post('/user/preferences', async (req, res) => {
  try {
    const { contentId, preferences } = req.body;
    
    // In a real implementation, this would save to the database
    // For now, return success
    res.status(200).json({
      success: true,
      message: `Preferences saved for content ${contentId}`,
      contentId,
      preferences
    });
  } catch (error) {
    console.error('Error saving preferences:', error);
    res.status(500).json({ error: 'Internal server error' });
  }
});

module.exports = router;