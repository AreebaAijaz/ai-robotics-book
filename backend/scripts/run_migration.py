"""Script to run database migrations."""

import sys
import time
from pathlib import Path

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from services.postgres import postgres_service


def wake_up_database(max_retries=8, delay=10):
    """Wake up Neon serverless database with retries.

    Neon databases go to sleep after inactivity and need time to wake up.
    This function sends simple queries until the database responds.
    """
    print("Waking up Neon database (this may take up to 60s for cold start)...")
    for attempt in range(max_retries):
        try:
            result = postgres_service.health_check()
            if result.get("status") == "connected":
                print(f"  Database ready (latency: {result.get('latency_ms')}ms)\n")
                return True
            elif result.get("error"):
                error_msg = result.get("error", "")
                # Print shortened error for visibility
                if len(error_msg) > 80:
                    error_msg = error_msg[:80] + "..."
                print(f"  Attempt {attempt + 1}/{max_retries}: {error_msg}")
        except Exception as e:
            print(f"  Attempt {attempt + 1}/{max_retries}: {str(e)[:60]}...")

        if attempt < max_retries - 1:
            print(f"    Waiting {delay}s for cold start...")
            time.sleep(delay)

    print("  Warning: Could not verify database connection, proceeding anyway...\n")
    return False


def run_with_retry(func, max_retries=3, delay=3):
    """Run a function with retry logic for cold starts."""
    last_error = None
    for attempt in range(max_retries):
        try:
            return func()
        except Exception as e:
            last_error = e
            if attempt < max_retries - 1:
                print(f"    Retry {attempt + 1}/{max_retries - 1} in {delay}s...")
                time.sleep(delay)
    raise last_error


def run_all_migrations():
    """Run all database migrations in order."""
    # Wake up Neon database first (handles cold start)
    wake_up_database()

    migrations_dir = Path(__file__).parent / "migrations"

    # Get all SQL files sorted by name
    migration_files = sorted(migrations_dir.glob("*.sql"))

    if not migration_files:
        print("No migration files found.")
        return

    print(f"Found {len(migration_files)} migration(s) to run.\n")

    for migration_path in migration_files:
        print(f"Running: {migration_path.name}")
        try:
            run_with_retry(
                lambda p=migration_path: postgres_service.run_migration(str(p)),
                max_retries=3,
                delay=3
            )
            print(f"  [OK] {migration_path.name} completed\n")
        except Exception as e:
            print(f"  [FAIL] {migration_path.name} failed: {e}\n")
            raise

    print("All migrations completed successfully!")


def main():
    """Run the database migrations."""
    try:
        run_all_migrations()
    except Exception as e:
        print(f"\nMigration failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
