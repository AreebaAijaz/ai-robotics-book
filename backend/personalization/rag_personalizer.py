"""RAG Personalizer - Generate personalized system prompts based on user profile."""

from typing import Optional

from services.user_service import user_service
from models.user import UserProfileResponse


# Persona templates for different expertise levels
EXPERTISE_PERSONAS = {
    "beginner": """You are explaining to someone new to robotics and AI. Use:
- Simple, everyday analogies
- Step-by-step explanations
- Avoid jargon or define technical terms when used
- Focus on concepts before details
- Use encouraging language""",

    "intermediate": """You are explaining to someone with foundational knowledge. Use:
- Technical terms with brief clarifications
- Build on common concepts
- Include some mathematical notation when helpful
- Connect ideas to practical applications
- Reference related topics they might explore""",

    "advanced": """You are explaining to a professional with strong technical background. Use:
- Precise technical terminology
- Mathematical formulations where appropriate
- Reference academic concepts and research
- Discuss trade-offs and design decisions
- Connect to industry practices""",

    "expert": """You are discussing with a fellow expert in the field. Use:
- Advanced technical terminology without simplification
- Detailed mathematical derivations
- Reference cutting-edge research and papers
- Discuss open problems and debates
- Provide nuanced analysis of approaches""",
}

# Learning goal modifiers
LEARNING_GOAL_MODIFIERS = {
    "casual": "Keep explanations concise and engaging. Focus on the most interesting aspects.",
    "deep_understanding": "Provide thorough explanations with underlying principles and theory.",
    "practical": "Emphasize hands-on applications, code examples, and implementation details.",
    "research": "Include academic references, current research trends, and open problems.",
}

# Time availability modifiers
TIME_MODIFIERS = {
    "quick_5min": "Be very concise. Provide the key points in 2-3 sentences, with an optional brief elaboration.",
    "medium_15min": "Provide a balanced explanation with key concepts and some details.",
    "deep_30min": "Provide comprehensive explanations with examples, context, and related topics.",
}

# Format preference modifiers
FORMAT_MODIFIERS = {
    "text": "Focus on clear written explanations with well-structured paragraphs.",
    "diagrams": "When helpful, suggest or describe visual representations and diagrams.",
    "videos": "When relevant, mention video resources or describe concepts visually.",
    "code": "Include code examples and pseudocode when explaining implementations.",
    "interactive": "Suggest hands-on exercises or experiments when applicable.",
}


class RAGPersonalizer:
    """Service for generating personalized RAG system prompts."""

    def get_user_context(self, user_id: str) -> Optional[UserProfileResponse]:
        """Fetch user profile for personalization.

        Args:
            user_id: User's unique identifier.

        Returns:
            UserProfileResponse or None if not found.
        """
        return user_service.get_profile(user_id)

    def generate_personalized_prompt(
        self,
        profile: Optional[UserProfileResponse],
        base_prompt: str = ""
    ) -> str:
        """Generate a personalized system prompt based on user profile.

        Args:
            profile: User's profile data.
            base_prompt: Base system prompt to enhance.

        Returns:
            Personalized system prompt string.
        """
        if not profile:
            return base_prompt

        # Build personalization context
        parts = []

        # Add expertise persona
        expertise = profile.expertise_level
        if expertise in EXPERTISE_PERSONAS:
            parts.append(f"## Communication Style\n{EXPERTISE_PERSONAS[expertise]}")

        # Add learning goal modifier
        goal = profile.learning_goals
        if goal in LEARNING_GOAL_MODIFIERS:
            parts.append(f"## Learning Approach\n{LEARNING_GOAL_MODIFIERS[goal]}")

        # Add time modifier
        time_pref = profile.time_availability
        if time_pref in TIME_MODIFIERS:
            parts.append(f"## Response Length\n{TIME_MODIFIERS[time_pref]}")

        # Add format preference
        format_pref = profile.preferred_format
        if format_pref in FORMAT_MODIFIERS:
            parts.append(f"## Content Format\n{FORMAT_MODIFIERS[format_pref]}")

        # Add interests context
        if profile.interests:
            interests_str = ", ".join(profile.interests)
            parts.append(f"## User Interests\nThe user is particularly interested in: {interests_str}. When relevant, connect explanations to these areas.")

        # Add challenges context
        if profile.challenges:
            parts.append(f"## User Challenges\nThe user mentioned: \"{profile.challenges}\". Be mindful of these challenges and provide helpful guidance when relevant.")

        # Combine parts
        personalization = "\n\n".join(parts)

        if base_prompt:
            return f"{base_prompt}\n\n# User Personalization\n\n{personalization}"
        else:
            return f"# User Personalization\n\n{personalization}"

    def get_personalized_system_prompt(
        self,
        user_id: Optional[str],
        base_prompt: str = ""
    ) -> str:
        """Get a complete personalized system prompt for a user.

        Args:
            user_id: User's unique identifier (None for anonymous).
            base_prompt: Base system prompt to enhance.

        Returns:
            Personalized system prompt string.
        """
        if not user_id:
            return base_prompt

        profile = self.get_user_context(user_id)
        return self.generate_personalized_prompt(profile, base_prompt)


# Singleton instance
rag_personalizer = RAGPersonalizer()
