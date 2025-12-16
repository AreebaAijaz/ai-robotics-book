"""Script to run database migrations."""

import sys
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from services.postgres import postgres_service


def main():
    """Run the database migration."""
    migration_path = Path(__file__).parent / "migrations" / "001_create_tables.sql"

    print(f"Running migration: {migration_path}")

    try:
        postgres_service.run_migration(str(migration_path))
        print("Migration completed successfully!")
    except Exception as e:
        print(f"Migration failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
