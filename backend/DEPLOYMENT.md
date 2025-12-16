# RAG Chatbot Backend - Render Deployment Guide

## Prerequisites

- GitHub account with the repository pushed
- Render account (free tier available): https://render.com
- Environment variables ready:
  - `OPENAI_API_KEY`
  - `QDRANT_URL`
  - `QDRANT_API_KEY`
  - `DATABASE_URL` (optional, for Neon Postgres analytics)

---

## Step 1: Push Backend to GitHub

Make sure your backend code is pushed to GitHub:

```bash
cd backend
git add .
git commit -m "Prepare backend for Render deployment"
git push origin main
```

---

## Step 2: Create Render Account

1. Go to https://render.com
2. Sign up with GitHub (recommended for easy repo access)
3. Verify your email

---

## Step 3: Create New Web Service

### Option A: Using render.yaml (Blueprint)

1. Go to Render Dashboard → **Blueprints**
2. Click **New Blueprint Instance**
3. Connect your GitHub repository
4. Select the repository containing the `backend/` folder
5. Render will detect `render.yaml` and configure automatically
6. Click **Apply**

### Option B: Manual Setup

1. Go to Render Dashboard → **New** → **Web Service**
2. Connect your GitHub account if not already connected
3. Select your repository
4. Configure the service:

| Setting | Value |
|---------|-------|
| **Name** | `rag-chatbot-api` |
| **Region** | Oregon (US West) or your preference |
| **Branch** | `main` (or your branch) |
| **Root Directory** | `backend` |
| **Runtime** | Python 3 |
| **Build Command** | `pip install -r requirements.txt` |
| **Start Command** | `uvicorn main:app --host 0.0.0.0 --port $PORT` |
| **Plan** | Free (for development) |

5. Click **Create Web Service**

---

## Step 4: Add Environment Variables

In the Render Dashboard for your service:

1. Go to **Environment** tab
2. Add the following variables:

| Key | Value | Notes |
|-----|-------|-------|
| `OPENAI_API_KEY` | `sk-proj-...` | Your OpenAI API key |
| `QDRANT_URL` | `https://xxx.cloud.qdrant.io` | Qdrant Cloud URL |
| `QDRANT_API_KEY` | `your-qdrant-key` | Qdrant API key |
| `DATABASE_URL` | `postgresql://...` | Neon Postgres URL (optional) |
| `CORS_ORIGINS` | `https://areebaaijaz.github.io,http://localhost:3000` | Allowed origins |
| `PYTHON_VERSION` | `3.11` | Python version |

3. Click **Save Changes**
4. The service will redeploy automatically

---

## Step 5: Get Your API URL

After deployment:

1. Go to your service in Render Dashboard
2. Find the URL at the top (e.g., `https://rag-chatbot-api.onrender.com`)
3. Test the health endpoint: `https://rag-chatbot-api.onrender.com/api/health`

---

## Step 6: Update Frontend

Update the production API URL in your frontend:

**File:** `ai-robotics-book/src/services/apiClient.js`

```javascript
// Update this line with your actual Render URL
const PRODUCTION_API_URL = 'https://rag-chatbot-api.onrender.com/api';
```

Then rebuild and deploy your frontend to GitHub Pages.

---

## Keeping the Service Warm (Avoiding Cold Starts)

Render's free tier spins down after 15 minutes of inactivity, causing cold starts (30-60 second delays).

### Option 1: Upgrade to Paid Plan ($7/month)
- Services never spin down
- Best for production use

### Option 2: Use a Cron Job Service (Free)

Use a free service like **cron-job.org** or **UptimeRobot**:

1. Go to https://cron-job.org (free account)
2. Create a new cron job:
   - **URL:** `https://rag-chatbot-api.onrender.com/api/health`
   - **Schedule:** Every 14 minutes
   - **Method:** GET

This keeps the service warm by pinging it regularly.

### Option 3: GitHub Actions (Free)

Add this workflow to your repo at `.github/workflows/keep-alive.yml`:

```yaml
name: Keep Render Alive

on:
  schedule:
    - cron: '*/14 * * * *'  # Every 14 minutes

jobs:
  ping:
    runs-on: ubuntu-latest
    steps:
      - name: Ping API
        run: curl -s https://rag-chatbot-api.onrender.com/api/health
```

---

## Troubleshooting

### Service won't start
- Check the **Logs** tab in Render Dashboard
- Verify all environment variables are set
- Ensure `requirements.txt` is in the `backend/` folder

### CORS errors
- Verify `CORS_ORIGINS` includes your frontend URL
- Make sure there are no trailing slashes in origins

### API returns 500 errors
- Check logs for Python errors
- Verify Qdrant and OpenAI credentials are correct

### Cold start delays
- First request after inactivity takes 30-60 seconds
- Use the keep-alive solutions above

---

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/` | GET | Root - returns API status |
| `/api/health` | GET | Health check with dependencies |
| `/api/chat` | POST | Send chat message |
| `/api/chat/selection` | POST | Send selection query |

---

## Cost Estimate

| Tier | Price | Features |
|------|-------|----------|
| **Free** | $0/month | 750 hours/month, cold starts |
| **Starter** | $7/month | Always on, no cold starts |
| **Standard** | $25/month | More resources, auto-scaling |

For a demo/portfolio project, the free tier with keep-alive is sufficient.
