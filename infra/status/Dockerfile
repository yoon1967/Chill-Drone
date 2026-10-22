FROM python:3.13-alpine@sha256:2d9aefe2fef018a7eb2c13064c89c71929800fd2e5dccdbf52ea5da5bb8d929a
ARG REVISION=unknown
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 REVISION=${REVISION}
WORKDIR /app
COPY server.py index.html status.css status.js favicon.svg ./
USER 65532:65532
EXPOSE 18066
CMD ["python", "server.py"]
