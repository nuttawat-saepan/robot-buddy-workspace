import { FileBlob, SpreadsheetFile } from "@oai/artifact-tool";

const filePath = "C:/Users/nutta/Desktop/projects/Systronic-20260722T074952Z-1-001/outputs/go2_field_test_checklist/go2_field_test_checklist.xlsx";
const input = await FileBlob.load(filePath);
const workbook = await SpreadsheetFile.importXlsx(input);

const sheets = await workbook.inspect({
  kind: "sheet",
  include: "id,name",
  maxChars: 4000,
});
console.log(sheets.ndjson);

const overview = await workbook.inspect({
  kind: "table",
  range: "Overview!A13:D18",
  include: "values,formulas",
  tableMaxRows: 10,
  tableMaxCols: 4,
});
console.log(overview.ndjson);

const checklist = await workbook.inspect({
  kind: "table",
  range: "Phase Checklist!A4:I12",
  include: "values,formulas",
  tableMaxRows: 12,
  tableMaxCols: 9,
});
console.log(checklist.ndjson);

const errors = await workbook.inspect({
  kind: "match",
  searchTerm: "#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A",
  options: { useRegex: true, maxResults: 100 },
  summary: "imported workbook formula error scan",
});
console.log(errors.ndjson);
