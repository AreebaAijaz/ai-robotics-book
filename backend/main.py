"""FastAPI application entry point for RAG Chatbot backend."""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from config import settings
from routers.chat import router as chat_router
from routers.auth import router as auth_router
from routers.user import router as user_router

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the Physical AI book RAG chatbot with authentication and personalization",
    version="2.0.0",
)

# Configure CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Include routers
app.include_router(auth_router)
app.include_router(user_router)
app.include_router(chat_router)


@app.get("/")
async def root():
    """Root endpoint."""
    return {"message": "RAG Chatbot API", "status": "running"}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
