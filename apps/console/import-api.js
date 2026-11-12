export async function requestImport(action, body, query = '', signal) {
  const options = body === undefined ? {cache: 'no-store'} : {
    method: 'POST',
    headers: {'X-Mapping-Client': '1', 'Content-Type': 'application/json'},
    body: JSON.stringify(body),
  };
  const response = await fetch(`/api/import/${action}${query}`, {...options, signal});
  const value = await response.json();
  if (!response.ok) {
    const error = new Error(value.error, {cause: value.error});
    error.httpStatus = response.status;
    throw error;
  }
  return value;
}
