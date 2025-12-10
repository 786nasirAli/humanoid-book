## User Authentication & Personalization Setup

This document explains how to set up and run the user authentication and personalization features of the course.

### Authentication Features

1. **User Registration/Login**: Secure authentication using better-auth.com
2. **Background Assessment**: Questionnaire to understand user's experience level
3. **Personalized Content**: Adaptation of content based on user preferences and background
4. **Progress Tracking**: Saving user progress across different modules

### Personalization Features

1. **Skill-Level Adaptation**: Content adjusted based on beginner, intermediate, or advanced levels
2. **Learning Style Preferences**: Options for detailed explanations, concise summaries, or example-heavy content
3. **Content Format Selection**: Choice of text, diagrams, or video content
4. **Language Preferences**: Support for English and Urdu languages

### Implementation Architecture

The authentication and personalization system is implemented in two parts:

1. **Frontend Components** (Docusaurus):
   - AuthProvider and authentication UI components
   - Personalization controls integrated into content pages
   - User profile management interface

2. **Backend Services** (Separate Express server):
   - User registration and login endpoints
   - Profile management and preferences storage
   - Personalization logic based on user data

### Setup Instructions

1. **Configure Environment Variables**:
   Add these to your `.env` file in the server directory:
   ```
   BETTER_AUTH_SECRET=your-better-auth-secret
   BETTER_AUTH_URL=http://localhost:3000
   NEON_DB_URL=postgresql://username:password@ep-...region.neon.tech/dbname
   ```

2. **Add Personalization Controls to Content**:
   The personalization control is added to content pages using:
   ```markdown
   import ContentPersonalization from '@site/src/components/UserAuth/ContentPersonalization';

   <ContentPersonalization title="Page Title" />
   ```

3. **Run the Services**:
   - Start the backend server: `cd server && npm start`
   - Start the Docusaurus site: `npm start`

### Integration Points

The personalization system integrates with:
- The RAG chatbot to provide personalized responses
- Content rendering to adapt complexity based on user level
- Progress tracking to remember user preferences across sessions
- The overall course structure to customize learning paths

### Customization

You can extend the personalization system by:
- Adding more user preference categories in the questionnaire
- Creating adaptive content rendering based on user background
- Implementing custom learning paths based on user goals
- Adding multilingual support for content adaptation