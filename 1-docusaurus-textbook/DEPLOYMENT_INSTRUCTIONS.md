# Deployment Guide for AI-Humanoid-Robotics Textbook

## Overview
This guide provides step-by-step instructions to deploy the Docusaurus-based AI-Humanoid-Robotics textbook with chatbot and authentication features to Vercel.

## Project Structure
```
1-docusaurus-textbook/
├── backend/          # FastAPI backend with auth and chat
└── frontend/         # Docusaurus frontend with chat UI
```

## Frontend Deployment to Vercel

### Step 1: Prepare the Repository
1. Ensure you're on the correct branch: `812f424cf1000ec35529c3260cc9dba9531a7d31`
2. Make sure all changes are committed:
   ```bash
   git add .
   git commit -m "Prepare for deployment: clean up and fix configs"
   git push origin main
   ```

### Step 2: Deploy Frontend to Vercel
1. Go to https://vercel.com/dashboard
2. Click "Add New" → "Project"
3. Import your GitHub repository (`Ummay480/AI-Humanoid-Robotics`)
4. Configure the project:
   - **Root Directory**: `1-docusaurus-textbook/frontend`
   - **Framework**: Docusaurus (auto-detected)
   - **Build Command**: `npm run build`
   - **Output Directory**: `build`
   - **Install Command**: `npm install`

5. Set environment variables:
   ```
   REACT_APP_API_URL = [your-backend-api-url]
   REACT_APP_AUTH_BASE_URL = [your-backend-api-url]/api/auth
   ```

6. Click "Deploy"

### Step 3: Backend Deployment
The backend needs to be deployed separately. You can deploy it to:
- Railway: `https://railway.app`
- Heroku: `https://heroku.com`
- Any other Python/Node hosting platform

For FastAPI backend:
1. Go to your chosen platform
2. Connect to the GitHub repository
3. Set root directory to: `1-docusaurus-textbook/backend`
4. Set environment variables:
   ```
   DATABASE_URL = [your-database-url]
   QDRANT_URL = [your-qdrant-url]
   GEMINI_API_KEY = [your-api-key]
   JWT_SECRET_KEY = [your-secret-key]
   ```

### Step 4: Update Frontend Environment Variables
After deploying the backend, update the frontend environment variables in Vercel:
- `REACT_APP_API_URL` = your deployed backend URL
- `REACT_APP_AUTH_BASE_URL` = your deployed backend URL + `/api/auth`

## Verification Steps

### 1. Check Frontend Deployment
- Visit your Vercel URL (e.g., `https://ai-humanoid-robotics-[hash].vercel.app`)
- Verify the site loads with:
  - All textbook chapters in the sidebar
  - Working navigation
  - Chat button visible in bottom-right corner

### 2. Check Backend Deployment
- Visit `https://[your-backend-url]/health` to verify backend is running
- Visit `https://[your-backend-url]/docs` for API documentation

### 3. Test Chat Functionality
- Click the chat button to open the chat panel
- Try registering/logging in (authentication should work)
- Send a test message to verify chat functionality

## Troubleshooting

### If the site doesn't load:
1. Check Vercel build logs for errors
2. Verify the root directory is set to `1-docusaurus-textbook/frontend`
3. Ensure all dependencies are properly installed

### If chat doesn't work:
1. Verify backend is deployed and accessible
2. Check that environment variables are correctly set
3. Confirm CORS settings allow your frontend domain

### If authentication doesn't work:
1. Verify auth endpoints are accessible
2. Check that JWT configuration is correct
3. Ensure session management is working

## Custom Domain Setup (Optional)
1. In Vercel dashboard → your project → Settings → Domains
2. Add your custom domain (e.g., `ai-humanoid-robotics.com`)
3. Update DNS settings as instructed by Vercel

## Maintenance
- Regularly update dependencies
- Monitor build logs for issues
- Keep API keys secure and rotate periodically
- Test functionality after each deployment

## Environment Variables Reference

### Frontend (Vercel):
- `REACT_APP_API_URL`: Backend API URL (e.g., `https://your-backend.onrender.com`)
- `REACT_APP_AUTH_BASE_URL`: Backend auth URL (e.g., `https://your-backend.onrender.com/api/auth`)

### Backend (Hosting Platform):
- `DATABASE_URL`: PostgreSQL database URL
- `QDRANT_URL`: Vector database URL
- `GEMINI_API_KEY`: Google Gemini API key
- `JWT_SECRET_KEY`: Secret key for JWT tokens
- `ENVIRONMENT`: `production` or `development`
- `DEBUG`: `true` or `false`