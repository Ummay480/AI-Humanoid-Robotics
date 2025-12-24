# Vercel Deployment Guide

## ✅ Pre-Deployment Checklist (COMPLETED)

- ✅ Docusaurus project structure verified at: `1-docusaurus-textbook/frontend`
- ✅ Configuration fixed: `docusaurus.config.js`
  - URL: `https://ai-humanoid-robotics-sepia.vercel.app`
  - BaseURL: `/`
- ✅ Package.json cleaned (removed non-existent dependencies)
- ✅ Build command: `npm run build` ✓
- ✅ Output directory: `build` ✓
- ✅ Local build successful ✓
- ✅ Code committed and pushed to `main` branch ✓

## 🚀 Deploy to Vercel (REQUIRED - Manual Steps)

### Step 1: Access Vercel Dashboard
1. Go to: https://vercel.com/dashboard
2. Log in with GitHub account: **Ummay480**

### Step 2: Import GitHub Repository
1. Click **"Add New"** → **"Project"**
2. Look for: **`Ummay480/AI-Humanoid-Robotics`**
3. Click **"Import"**

### Step 3: Configure Project Settings

**CRITICAL: Set these values exactly:**

```
Framework Preset: Docusaurus
Root Directory: 1-docusaurus-textbook/frontend
Build Command: npm run build
Output Directory: build
Install Command: npm install
Node.js Version: 20.x
```

### Step 4: Deploy

1. Click **"Deploy"** button
2. Wait for build to complete (~3-5 minutes)
3. Vercel will provide deployment URL

## 🧪 Expected Deployment URL

After successful deployment, your site will be available at:

**https://ai-humanoid-robotics-sepia.vercel.app/**

Or Vercel may assign a different URL like:
- `https://ai-humanoid-robotics-[hash].vercel.app`

## ✓ Verification Steps

Once deployed, verify these URLs work:

1. **Homepage**: https://ai-humanoid-robotics-sepia.vercel.app/
2. **Docs**: https://ai-humanoid-robotics-sepia.vercel.app/docs/intro
3. **Blog**: https://ai-humanoid-robotics-sepia.vercel.app/blog

## 🛑 Troubleshooting

### If build fails:
1. Check Vercel build logs
2. Verify Root Directory is: `1-docusaurus-textbook/frontend`
3. Ensure Node version is 18.x or 20.x

### If site shows 404:
1. Verify Output Directory is: `build`
2. Check that baseUrl in config is: `/`
3. Redeploy from Vercel dashboard

### If site loads but content is missing:
1. Check that all docs files exist in `/docs` directory
2. Verify sidebar configuration in `sidebars.js`

## 📝 Build Information

**Last Successful Local Build:**
- Date: 2025-12-24
- Build Time: ~4.5 minutes
- Output: `Generated static files in "build"`
- Status: ✅ SUCCESS

**Git Status:**
- Branch: `main`
- Last Commit: `cb160135` - fix(docusaurus): clean build for Vercel deployment
- Pushed: ✅ Yes

## 🎯 Next Steps After Deployment

1. Get the live URL from Vercel dashboard
2. Test all pages: homepage, docs, blog
3. Verify navigation works
4. Check that GitHub links in footer/navbar work
5. (Optional) Set up custom domain in Vercel settings

## 📞 Support

If deployment fails:
- Check Vercel build logs for specific errors
- Verify all configuration settings match this guide
- Ensure GitHub repository access is granted to Vercel
